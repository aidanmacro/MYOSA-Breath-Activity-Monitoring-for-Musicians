import csv
import socket
import struct
import sys
import threading
import time
from collections import deque

import numpy as np
import pyqtgraph as pg
from PyQt6 import QtCore, QtGui, QtWidgets

# --- UDP Configuration ---
UDP_IP = "0.0.0.0"  # Listen on all network adapters
UDP_PORT = 12345

# --- Packet configuration ---
EXPECTED_SAMPLES = 512
MAGIC_BYTES = b"OCIP!CDA"
# sequence, dropped, samples, checksum, temperature, pressure
HEADER_REST_FMT = "<I I H H f f"
HEADER_REST_SIZE = struct.calcsize(HEADER_REST_FMT)

# --- ADC scaling ---
VREF = 3.3
ADC_MAX = 4095.0
ADC_SAMPLE_RATE_HZ = 200000

# --- Rolling buffer sizes ---
ROLLING_ADC_SAMPLES = 8192
ROLLING_BARO_POINTS = 600     # ~10 min at 1 reading/sec
ROLLING_LED_POINTS = 600      # ~10 min of LED on/off + photocurrent history
ROLLING_CO2_POINTS = 4000     # ~2+ min of CO2 history at ~30 Hz

# --- 10 ms historical averaging window ---
LED_AVG_WINDOW_S = 0.10
LED_AVG_WINDOW_SAMPLES = int(LED_AVG_WINDOW_S * ADC_SAMPLE_RATE_HZ)  # 2000 samples

# --- TIA / analogue front-end model (see Figure 5.2 schematic) ---
TIA_RF_OHMS = 120e3               # Rf1, transimpedance feedback resistor
STAGE2_GAIN = 1 + (240e3 / 1e3)   # Av = 1 + Rf2/Rf3 = 241
TOTAL_TRANSIMPEDANCE = TIA_RF_OHMS * STAGE2_GAIN  # V per A

# Photosensitivity temperature coefficient of the InAsSb detector
# (~ -0.1 %/degC around the 4.3 um band, back-illuminated type, per datasheet)
ALPHA_PD_PER_DEGC = -0.001
T0_REF_DEGC = 25.0

# --- Detector responsivity, used to reverse photocurrent -> incident optical power ---
DETECTOR_RESPONSIVITY_A_PER_W = 4.5e-3  # S = 4.5 mA/W typ. at lambda_p

# --- Physical sanity clamp on photocurrent ---
# With the M16615-style driver pushing ~400 mA pulses through the L15895 LED
# and the P16112 detector's responsivity/geometry, a differential (on-off)
# photocurrent above ~2 nA is not physically achievable through this optical
# path. Anything larger is treated as a corrupted/glitched reading and is
# dropped rather than fed into the CO2 chain.
MAX_PHYSICAL_PHOTOCURRENT_NA = 2.0

# --- NDIR CO2 (Beer-Lambert) calibration defaults ---
# Engineering estimates, not a factory calibration -- calibrate against a
# known-CO2 reference gas for accurate absolute readings.
DEFAULT_PATH_LENGTH_CM = 3
DEFAULT_ABS_COEFF_PER_PCT_CM = 0.10

# --- Breath-control status thresholds (heuristic, tune to taste) ---
CO2_IDLE_PCT = 0.20
CO2_SUSTAIN_MIN_PCT = 1.00
CO2_OVERBLOW_PCT = 5.00
SLOPE_RISE_THRESH_PCT_S = 3.0
SLOPE_FALL_THRESH_PCT_S = -3.0
SLOPE_STEADY_BAND_PCT_S = 1.0
LEAK_STD_THRESH_PCT = 0.40
STATUS_WINDOW_S = 0.75

# --- Pressure-based secondary blow detection ---
# Blowing into the mouthpiece perturbs local pressure at the barometer much
# faster than CO2 can build up (CO2 relies on mixing/diffusion in the bore),
# so a pressure rise can flag "breath onset" a beat before CO2 confirms it.
# The absolute magnitude is hardware/mounting dependent -- tune via the UI.
DEFAULT_PRESSURE_BLOW_THRESHOLD_PA = 0.05

# --- Shared state (guarded by `lock`) ---
lock = threading.Lock()
running = True

rolling_volts = np.array([], dtype=np.float32)
rolling_pwm = np.array([], dtype=np.uint8)
latest_status = "Waiting..."
packet_counter = 0   # increments on every valid packet; lets consumers detect "no new data"

baro_t = deque(maxlen=ROLLING_BARO_POINTS)      # wall-clock seconds
baro_temp = deque(maxlen=ROLLING_BARO_POINTS)   # deg C
baro_pressure = deque(maxlen=ROLLING_BARO_POINTS)  # Pa
latest_temp = None
latest_pressure = None

# --- Shared wall-clock time-zero, used by every "Time (s)" plot (LED, ---
# --- Photocurrent, Temperature, Pressure) so their x-axes line up and ---
# --- can be panned/zoomed together instead of drifting apart.         ---
session_start_time = None


def get_session_start_time(now):
    """Return the shared t=0 reference, initialising it on first call."""
    global session_start_time
    with lock:
        if session_start_time is None:
            session_start_time = now
        return session_start_time


def checksum_u16(adc_u16):
    return int(np.sum(adc_u16, dtype=np.uint32) & 0xFFFF)


def udp_thread():
    global rolling_volts, rolling_pwm, latest_status
    global latest_temp, latest_pressure, packet_counter

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(0.1)

    expected_size = len(MAGIC_BYTES) + HEADER_REST_SIZE + (EXPECTED_SAMPLES * 2)

    try:
        while running:
            try:
                data, addr = sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break

            if len(data) != expected_size:
                continue

            if data[:len(MAGIC_BYTES)] != MAGIC_BYTES:
                continue

            packet_start = len(MAGIC_BYTES)
            rest = data[packet_start : packet_start + HEADER_REST_SIZE]
            raw = data[packet_start + HEADER_REST_SIZE :]

            (_sequence, _dropped, samples, checksum,
             temperature, pressure) = struct.unpack(HEADER_REST_FMT, rest)

            if samples != EXPECTED_SAMPLES:
                continue

            adc_u16 = np.frombuffer(raw, dtype="<u2").copy()
            if checksum_u16(adc_u16) != checksum:
                continue

            pwm_state = ((adc_u16 >> 15) & 0x1).astype(np.uint8)
            adc_u16 = adc_u16 & 0x0FFF
            volts = adc_u16.astype(np.float32) * (VREF / ADC_MAX)

            status = f"min={volts.min():.3f} V | max={volts.max():.3f} V | Dropped={_dropped}"
            now = time.time()
            t0 = get_session_start_time(now)

            with lock:
                rolling_volts = np.concatenate((rolling_volts, volts))
                rolling_pwm = np.concatenate((rolling_pwm, pwm_state))
                if len(rolling_volts) > ROLLING_ADC_SAMPLES:
                    rolling_volts = rolling_volts[-ROLLING_ADC_SAMPLES:]
                    rolling_pwm = rolling_pwm[-ROLLING_ADC_SAMPLES:]

                latest_status = status

                baro_t.append(now - t0)
                baro_temp.append(float(temperature))
                baro_pressure.append(float(pressure))
                latest_temp = temperature
                latest_pressure = pressure
                packet_counter += 1

    finally:
        sock.close()


def find_trigger_window(signal, trace_length, level, edge, pretrigger_frac=0.25):
    """
    Scan `signal` for the most recent level-crossing that still leaves a full
    `trace_length` window available (with `pretrigger_frac` of it before the
    trigger point), mimicking a real oscilloscope's trigger + pre-trigger view.
    Returns (start, end, trigger_index) or None if no valid crossing is found.
    """
    n = len(signal)
    if n < 2 or trace_length < 2:
        return None

    pretrigger = int(trace_length * pretrigger_frac)

    if edge == "Rising":
        idxs = np.where((signal[:-1] < level) & (signal[1:] >= level))[0] + 1
    else:  # "Falling"
        idxs = np.where((signal[:-1] >= level) & (signal[1:] < level))[0] + 1

    for idx in idxs[::-1]:
        start = idx - pretrigger
        end = start + trace_length
        if start >= 0 and end <= n:
            return start, end, idx

    return None


class ScopeWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Trombone Mouthpiece CO2 / Breath Monitor")

        self.paused = False
        self.trace_length = 2048
        self.follow_latest = True

        self.last_plot_x = np.array([], dtype=np.float32)
        self.last_plot_y = np.array([], dtype=np.float32)
        self.last_plot_pwm = np.array([], dtype=np.uint8)

        # --- Trigger state ---
        self.trigger_enabled = False
        self.trigger_source = "PWM State"
        self.trigger_edge = "Rising"
        self.trigger_level = VREF / 2.0
        self.trigger_pretrigger_frac = 0.25

        # --- LED on/off average + photocurrent history ---
        # Note: no separate led_start_time -- uses the shared session_start_time
        # (via get_session_start_time) so this lines up with the Temp/Pressure
        # plots' time axis and the two can be scrolled/zoomed in sync.
        self.last_seen_packet_count = -1  # forces has_new_packet True on first tick
        self.led_t = deque(maxlen=ROLLING_LED_POINTS)
        self.led_on_v = deque(maxlen=ROLLING_LED_POINTS)
        self.led_off_v = deque(maxlen=ROLLING_LED_POINTS)
        self.photocurrent_na = deque(maxlen=ROLLING_LED_POINTS)  # temp-compensated, clamp-filtered

        self.power_smooth_window = deque(maxlen=5)

        # --- CO2 / baseline state ---
        self.baseline_power_uw = None
        self.baseline_pressure_pa = None
        self.co2_path_length_cm = DEFAULT_PATH_LENGTH_CM
        self.co2_abs_coeff = DEFAULT_ABS_COEFF_PER_PCT_CM
        self.co2_start_time = None
        self.co2_t = deque(maxlen=ROLLING_CO2_POINTS)
        self.co2_pct = deque(maxlen=ROLLING_CO2_POINTS)

        # --- Pressure-based secondary blow detection ---
        self.pressure_blow_threshold_pa = DEFAULT_PRESSURE_BLOW_THRESHOLD_PA
        self.pressure_invert = False

        # =========================================================
        # Root layout: two columns. LEFT = main CO2 display,
        # RIGHT = diagnostic scope traces (larger/more visible).
        # =========================================================
        root = QtWidgets.QHBoxLayout(self)

        left_col = QtWidgets.QVBoxLayout()
        right_col = QtWidgets.QVBoxLayout()
        root.addLayout(left_col, stretch=6)
        root.addLayout(right_col, stretch=5)

        # =========================================================
        # LEFT COLUMN: CO2 digital readout + breath status + big plot
        # =========================================================
        co2_group = QtWidgets.QGroupBox("CO2 Monitor (NDIR, 4.26 \u00b5m band)")
        co2_layout = QtWidgets.QVBoxLayout(co2_group)

        top_row = QtWidgets.QHBoxLayout()

        self.co2_digital_label = QtWidgets.QLabel("--.-- %")
        digital_font = QtGui.QFont("Consolas", 48, QtGui.QFont.Weight.Bold)
        self.co2_digital_label.setFont(digital_font)
        self.co2_digital_label.setStyleSheet(
            "background-color: black; color: #33ff33; padding: 8px; border-radius: 6px;"
        )
        self.co2_digital_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.co2_digital_label.setMinimumWidth(260)
        top_row.addWidget(self.co2_digital_label)

        status_col = QtWidgets.QVBoxLayout()
        self.breath_status_label = QtWidgets.QLabel("No Baseline / Awaiting Capture")
        status_font = QtGui.QFont("Arial", 16, QtGui.QFont.Weight.Bold)
        self.breath_status_label.setFont(status_font)
        self.breath_status_label.setStyleSheet(
            "background-color: #444444; color: white; padding: 6px; border-radius: 6px;"
        )
        self.breath_status_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        status_col.addWidget(self.breath_status_label)

        self.pressure_status_label = QtWidgets.QLabel("Pressure: n/a")
        status_col.addWidget(self.pressure_status_label)

        baseline_row = QtWidgets.QHBoxLayout()
        self.baseline_button = QtWidgets.QPushButton("Capture Baseline (Air)")
        self.baseline_button.clicked.connect(self.capture_baseline)
        baseline_row.addWidget(self.baseline_button)

        self.baseline_label = QtWidgets.QLabel("Baseline: not captured")
        baseline_row.addWidget(self.baseline_label)
        baseline_row.addStretch(1)
        status_col.addLayout(baseline_row)

        calib_row1 = QtWidgets.QHBoxLayout()
        calib_row1.addWidget(QtWidgets.QLabel("Path length (cm)"))
        self.path_length_spin = QtWidgets.QDoubleSpinBox()
        self.path_length_spin.setRange(0.1, 20.0)
        self.path_length_spin.setSingleStep(0.1)
        self.path_length_spin.setValue(self.co2_path_length_cm)
        self.path_length_spin.setToolTip(
            "LED-to-detector separation across the mouthpiece bore."
        )
        self.path_length_spin.valueChanged.connect(self.update_path_length)
        calib_row1.addWidget(self.path_length_spin)

        calib_row1.addWidget(QtWidgets.QLabel("Abs. coeff (per %CO2\u00b7cm)"))
        self.abs_coeff_spin = QtWidgets.QDoubleSpinBox()
        self.abs_coeff_spin.setRange(0.01, 10.0)
        self.abs_coeff_spin.setSingleStep(0.05)
        self.abs_coeff_spin.setValue(self.co2_abs_coeff)
        self.abs_coeff_spin.setToolTip(
            "Beer-Lambert absorption coefficient at 4.26 um. Calibrate against "
            "a known CO2 reference gas for accurate readings."
        )
        self.abs_coeff_spin.valueChanged.connect(self.update_abs_coeff)
        calib_row1.addWidget(self.abs_coeff_spin)
        calib_row1.addStretch(1)
        status_col.addLayout(calib_row1)

        calib_row2 = QtWidgets.QHBoxLayout()
        calib_row2.addWidget(QtWidgets.QLabel("Pressure blow thresh. (Pa)"))
        self.pressure_thresh_spin = QtWidgets.QDoubleSpinBox()
        self.pressure_thresh_spin.setRange(0.1, 200.0)
        self.pressure_thresh_spin.setSingleStep(0.5)
        self.pressure_thresh_spin.setValue(self.pressure_blow_threshold_pa)
        self.pressure_thresh_spin.setToolTip(
            "Pressure rise above the captured baseline that counts as 'blowing'. "
            "Hardware/mounting dependent -- tune to your enclosure."
        )
        self.pressure_thresh_spin.valueChanged.connect(self.update_pressure_threshold)
        calib_row2.addWidget(self.pressure_thresh_spin)

        self.pressure_invert_checkbox = QtWidgets.QCheckBox("Invert sign")
        self.pressure_invert_checkbox.setToolTip(
            "Enable if your barometer reads a pressure DROP when you blow "
            "(depends on sensor placement/mounting)."
        )
        self.pressure_invert_checkbox.stateChanged.connect(self.update_pressure_invert)
        calib_row2.addWidget(self.pressure_invert_checkbox)
        calib_row2.addStretch(1)
        status_col.addLayout(calib_row2)

        top_row.addLayout(status_col, stretch=1)
        co2_layout.addLayout(top_row)

        self.co2_plot_widget = pg.PlotWidget(title="CO2 % vs Time (since baseline capture)")
        self.co2_plot_widget.setLabel("bottom", "Time", units="s")
        self.co2_plot_widget.setLabel("left", "CO2", units="%")
        self.co2_plot_widget.showGrid(x=True, y=True)
        self.co2_curve = self.co2_plot_widget.plot(pen=pg.mkPen(width=2, color="#33ff33"))
        self.co2_idle_line = pg.InfiniteLine(
            pos=CO2_IDLE_PCT, angle=0,
            pen=pg.mkPen(color="#888888", width=1, style=QtCore.Qt.PenStyle.DashLine),
        )
        self.co2_sustain_line = pg.InfiniteLine(
            pos=CO2_SUSTAIN_MIN_PCT, angle=0,
            pen=pg.mkPen(color="#3399ff", width=1, style=QtCore.Qt.PenStyle.DashLine),
        )
        self.co2_plot_widget.addItem(self.co2_idle_line)
        self.co2_plot_widget.addItem(self.co2_sustain_line)
        self.co2_plot_widget.setMinimumHeight(420)
        co2_layout.addWidget(self.co2_plot_widget, stretch=1)

        left_col.addWidget(co2_group, stretch=1)

        # =========================================================
        # RIGHT COLUMN: diagnostic scope traces, stacked vertically
        # so each one gets full column width and is easy to read.
        # =========================================================
        self.plot_widget = pg.GraphicsLayoutWidget()

        self.adc_plot = self.plot_widget.addPlot(row=0, col=0, title="Waiting for data...")
        self.adc_plot.setLabel("bottom", "Sample (rel. trigger)")
        self.adc_plot.setLabel("left", "Voltage", units="V")
        self.adc_plot.setYRange(0, VREF)
        self.adc_plot.showGrid(x=True, y=True)
        self.adc_curve = self.adc_plot.plot(pen=pg.mkPen(width=1))

        self.trigger_level_line = pg.InfiniteLine(
            pos=self.trigger_level, angle=0,
            pen=pg.mkPen(color="g", width=1, style=QtCore.Qt.PenStyle.DashLine),
            movable=False,
        )
        self.adc_plot.addItem(self.trigger_level_line)
        self.trigger_level_line.setVisible(False)

        self.trigger_point_line = pg.InfiniteLine(
            pos=0, angle=90,
            pen=pg.mkPen(color="g", width=1, style=QtCore.Qt.PenStyle.DashLine),
        )
        self.adc_plot.addItem(self.trigger_point_line)
        self.trigger_point_line.setVisible(False)

        self.pwm_plot = self.plot_widget.addPlot(row=1, col=0, title="PWM Pin State")
        self.pwm_plot.setLabel("bottom", "Sample (rel. trigger)")
        self.pwm_plot.setLabel("left", "State")
        self.pwm_plot.setYRange(-0.2, 1.2, padding=0)
        self.pwm_plot.getAxis("left").setTicks([[(0, "0"), (1, "1")]])
        self.pwm_plot.showGrid(x=True, y=True)
        self.pwm_plot.setXLink(self.adc_plot.getViewBox())
        self.pwm_curve = self.pwm_plot.plot(pen=pg.mkPen(width=2, color="y"), stepMode="right")

        self.led_plot = self.plot_widget.addPlot(row=2, col=0, title="LED On/Off Photodiode Voltage (10 ms avg)")
        self.led_plot.setLabel("bottom", "Time", units="s")
        self.led_plot.setLabel("left", "Voltage", units="V")
        self.led_plot.showGrid(x=True, y=True)
        self.led_on_curve = self.led_plot.plot(pen=pg.mkPen(width=2, color="m"), name="LED On")
        self.led_off_curve = self.led_plot.plot(pen=pg.mkPen(width=2, color="w"), name="LED Off")
        self.led_plot.addLegend()

        self.photocurrent_plot = self.plot_widget.addPlot(row=3, col=0, title="Photocurrent (clamped to \u00b12 nA)")
        self.photocurrent_plot.setLabel("bottom", "Time", units="s")
        self.photocurrent_plot.setLabel("left", "Photocurrent", units="nA")
        self.photocurrent_plot.showGrid(x=True, y=True)
        self.photocurrent_plot.setXLink(self.led_plot.getViewBox())
        self.photocurrent_curve = self.photocurrent_plot.plot(pen=pg.mkPen(width=2, color="g"))

        self.temp_plot = self.plot_widget.addPlot(row=4, col=0, title="Temperature")
        self.temp_plot.setXLink(self.led_plot.getViewBox())
        self.temp_plot.setLabel("bottom", "Time", units="s")
        self.temp_plot.setLabel("left", "Temp", units="\u00b0C")
        self.temp_plot.showGrid(x=True, y=True)
        self.temp_curve = self.temp_plot.plot(pen=pg.mkPen(width=2, color="r"), symbol="o", symbolSize=4)

        self.pressure_plot = self.plot_widget.addPlot(row=5, col=0, title="Pressure (with blow-detect baseline)")
        self.pressure_plot.setLabel("bottom", "Time", units="s")
        self.pressure_plot.setLabel("left", "Pressure", units="Pa")
        self.pressure_plot.showGrid(x=True, y=True)
        self.pressure_plot.setXLink(self.temp_plot.getViewBox())
        self.pressure_curve = self.pressure_plot.plot(pen=pg.mkPen(width=2, color="c"), symbol="o", symbolSize=4)
        self.pressure_baseline_line = pg.InfiniteLine(
            pos=0, angle=0,
            pen=pg.mkPen(color="#888888", width=1, style=QtCore.Qt.PenStyle.DashLine),
        )
        self.pressure_plot.addItem(self.pressure_baseline_line)
        self.pressure_baseline_line.setVisible(False)

        # Give every diagnostic trace equal, generous vertical room.
        for r in range(6):
            self.plot_widget.ci.layout.setRowStretchFactor(r, 1)

        right_col.addWidget(self.plot_widget, stretch=1)

        # --- Controls (right column): pause / export / follow / trace length ---
        controls = QtWidgets.QGridLayout()

        self.pause_button = QtWidgets.QPushButton("Pause")
        self.pause_button.clicked.connect(self.toggle_pause)
        controls.addWidget(self.pause_button, 0, 0)

        self.export_button = QtWidgets.QPushButton("Export CSV")
        self.export_button.clicked.connect(self.export_csv)
        controls.addWidget(self.export_button, 0, 1)

        self.follow_checkbox = QtWidgets.QCheckBox("Follow latest")
        self.follow_checkbox.setChecked(True)
        self.follow_checkbox.stateChanged.connect(self.update_follow_latest)
        controls.addWidget(self.follow_checkbox, 0, 2)

        controls.addWidget(QtWidgets.QLabel("ADC trace length"), 1, 0)
        self.trace_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.trace_slider.setRange(64, ROLLING_ADC_SAMPLES)
        self.trace_slider.setSingleStep(64)
        self.trace_slider.setPageStep(512)
        self.trace_slider.setValue(self.trace_length)
        self.trace_slider.valueChanged.connect(self.update_trace_length)
        controls.addWidget(self.trace_slider, 1, 1, 1, 2)

        self.trace_label = QtWidgets.QLabel(f"{self.trace_length}")
        controls.addWidget(self.trace_label, 1, 3)

        self.status_label = QtWidgets.QLabel("Temp: n/a | Pressure: n/a")
        controls.addWidget(self.status_label, 2, 0, 1, 4)

        self.led_status_label = QtWidgets.QLabel("LED On: n/a | LED Off: n/a | Photocurrent: n/a")
        controls.addWidget(self.led_status_label, 3, 0, 1, 4)

        right_col.addLayout(controls)

        # --- Trigger controls ---
        trigger_box = QtWidgets.QGroupBox("Trigger")
        trigger_layout = QtWidgets.QGridLayout(trigger_box)

        self.trigger_enable_checkbox = QtWidgets.QCheckBox("Enable")
        self.trigger_enable_checkbox.stateChanged.connect(self.update_trigger_enabled)
        trigger_layout.addWidget(self.trigger_enable_checkbox, 0, 0)

        trigger_layout.addWidget(QtWidgets.QLabel("Source"), 0, 1)
        self.trigger_source_combo = QtWidgets.QComboBox()
        self.trigger_source_combo.addItems(["PWM State", "ADC Voltage"])
        self.trigger_source_combo.currentTextChanged.connect(self.update_trigger_source)
        trigger_layout.addWidget(self.trigger_source_combo, 0, 2)

        trigger_layout.addWidget(QtWidgets.QLabel("Edge"), 1, 0)
        self.trigger_edge_combo = QtWidgets.QComboBox()
        self.trigger_edge_combo.addItems(["Rising", "Falling"])
        self.trigger_edge_combo.currentTextChanged.connect(self.update_trigger_edge)
        trigger_layout.addWidget(self.trigger_edge_combo, 1, 1)

        trigger_layout.addWidget(QtWidgets.QLabel("Level (V)"), 1, 2)
        self.trigger_level_spin = QtWidgets.QDoubleSpinBox()
        self.trigger_level_spin.setRange(0.0, VREF)
        self.trigger_level_spin.setSingleStep(0.05)
        self.trigger_level_spin.setValue(self.trigger_level)
        self.trigger_level_spin.setEnabled(False)
        self.trigger_level_spin.valueChanged.connect(self.update_trigger_level)
        trigger_layout.addWidget(self.trigger_level_spin, 1, 3)

        right_col.addWidget(trigger_box)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(33)

    # --- UI callbacks: scope controls ---
    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_button.setText("Run" if self.paused else "Pause")

    def update_follow_latest(self, state):
        self.follow_latest = state == QtCore.Qt.CheckState.Checked.value

    def update_trace_length(self, value):
        value = max(64, int(value))
        self.trace_length = value
        self.trace_label.setText(f"{value}")

    def update_trigger_enabled(self, state):
        self.trigger_enabled = state == QtCore.Qt.CheckState.Checked.value
        self.trigger_point_line.setVisible(self.trigger_enabled)
        self.trigger_level_line.setVisible(self.trigger_enabled and self.trigger_source == "ADC Voltage")

    def update_trigger_source(self, text):
        self.trigger_source = text
        is_adc = text == "ADC Voltage"
        self.trigger_level_spin.setEnabled(is_adc)
        self.trigger_level_line.setVisible(self.trigger_enabled and is_adc)

    def update_trigger_edge(self, text):
        self.trigger_edge = text

    def update_trigger_level(self, value):
        self.trigger_level = float(value)
        self.trigger_level_line.setPos(self.trigger_level)

    # --- UI callbacks: CO2 calibration / baseline / pressure ---
    def update_path_length(self, value):
        self.co2_path_length_cm = float(value)

    def update_abs_coeff(self, value):
        self.co2_abs_coeff = float(value)

    def update_pressure_threshold(self, value):
        self.pressure_blow_threshold_pa = float(value)

    def update_pressure_invert(self, state):
        self.pressure_invert = state == QtCore.Qt.CheckState.Checked.value

    def capture_baseline(self):
        if not self.power_smooth_window:
            QtWidgets.QMessageBox.warning(
                self, "Capture Baseline",
                "No photocurrent data yet -- wait for the LED/detector to start streaming."
            )
            return

        self.baseline_power_uw = float(np.mean(self.power_smooth_window))

        with lock:
            p_now = latest_pressure

        self.baseline_pressure_pa = float(p_now) if p_now is not None else None

        baseline_text = f"Baseline: {self.baseline_power_uw:.4f} \u00b5W"
        if self.baseline_pressure_pa is not None:
            baseline_text += f" @ {self.baseline_pressure_pa:.1f} Pa (captured on air)"
        self.baseline_label.setText(baseline_text)

        self.pressure_baseline_line.setPos(0)
        self.pressure_baseline_line.setVisible(True)

        # Zero the CO2 timeline/history so the main display starts fresh
        # relative to this new reference.
        self.co2_start_time = time.time()
        self.co2_t.clear()
        self.co2_pct.clear()
        self.co2_curve.setData([], [])

    def export_csv(self):
        if not self.paused:
            QtWidgets.QMessageBox.warning(self, "Export CSV", "Pause the trace before exporting.")
            return

        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Export data", "capture.csv", "CSV files (*.csv)")
        if not path:
            return

        base = path[:-4] if path.lower().endswith(".csv") else path

        if len(self.last_plot_x) and len(self.last_plot_y):
            with open(base + "_adc.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["sample", "time_s", "voltage_v", "pwm_state"])
                for i, (sample, voltage) in enumerate(zip(self.last_plot_x, self.last_plot_y)):
                    pwm_val = int(self.last_plot_pwm[i]) if i < len(self.last_plot_pwm) else ""
                    writer.writerow([
                        int(sample), float(sample) / float(ADC_SAMPLE_RATE_HZ), float(voltage), pwm_val,
                    ])

        with lock:
            t = list(baro_t)
            temp = list(baro_temp)
            pressure = list(baro_pressure)

        if t:
            with open(base + "_baro.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["time_s", "temperature_c", "pressure_pa"])
                for ti, tempi, pi in zip(t, temp, pressure):
                    writer.writerow([ti, tempi, pi])

        if self.led_t:
            with open(base + "_photocurrent.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["time_s", "led_on_v", "led_off_v", "photocurrent_na"])
                for ti, von, voff, ipd in zip(self.led_t, self.led_on_v, self.led_off_v, self.photocurrent_na):
                    writer.writerow([ti, von, voff, ipd])

        if self.co2_t:
            with open(base + "_co2.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["time_s", "co2_pct"])
                for ti, c in zip(self.co2_t, self.co2_pct):
                    writer.writerow([ti, c])

    # --- LED on/off averaging + photocurrent + optical power + CO2 ---
    def compute_led_averages(self, volts_full, pwm_full):
        """10 ms rolling average of ADC voltage, split by PWM (LED) state."""
        if len(volts_full) == 0:
            return None, None

        n = min(len(volts_full), LED_AVG_WINDOW_SAMPLES)
        window_v = volts_full[-n:]
        window_pwm = pwm_full[-n:]

        on_mask = window_pwm == 1
        off_mask = window_pwm == 0

        led_on_avg = float(window_v[on_mask].mean()) if np.any(on_mask) else None
        led_off_avg = float(window_v[off_mask].mean()) if np.any(off_mask) else None
        return led_on_avg, led_off_avg

    def compute_photocurrent_na(self, led_on_avg, led_off_avg, temperature_c):
        """
        Photocurrent from the TIA + 2nd stage gain. The LED-off sample serves
        as the instantaneous baseline (cancels Vref/offset/bias-current
        drift), then the result is temperature-compensated back to a
        25 degC-equivalent reading. Finally, any reading exceeding what's
        physically achievable through this LED/detector pair is rejected
        (returns None) rather than corrupting the CO2 chain.
        """
        if led_on_avg is None or led_off_avg is None:
            return None

        delta_v = led_on_avg - led_off_avg
        i_pd_raw = delta_v / TOTAL_TRANSIMPEDANCE  # amps

        if temperature_c is not None:
            denom = 1.0 + ALPHA_PD_PER_DEGC * (temperature_c - T0_REF_DEGC)
            if denom != 0:
                i_pd_raw = i_pd_raw / denom

        i_pd_na = i_pd_raw * 1e9

        if abs(i_pd_na) > MAX_PHYSICAL_PHOTOCURRENT_NA:
            return None  # physically impossible given LED drive current -- discard

        return i_pd_na

    def photocurrent_to_optical_power_uw(self, i_pd_na):
        """Reverse the detector responsivity S = I_pd / P_incident."""
        if i_pd_na is None:
            return None
        i_pd_a = i_pd_na * 1e-9
        p_w = i_pd_a / DETECTOR_RESPONSIVITY_A_PER_W
        return p_w * 1e6  # microwatts

    def compute_co2_percent(self, p_measured_uw):
        """
        Beer-Lambert: T = P/P0, A = -ln(T), CO2% = A / (abs_coeff * path_length).
        P0 (baseline) is captured on ambient air.
        """
        if p_measured_uw is None or self.baseline_power_uw is None:
            return None
        if p_measured_uw <= 0 or self.baseline_power_uw <= 0:
            return None

        transmittance = p_measured_uw / self.baseline_power_uw
        if transmittance <= 0:
            return None

        absorbance = -np.log(transmittance)
        denom = self.co2_abs_coeff * self.co2_path_length_cm
        if denom == 0:
            return None

        co2_pct = absorbance / denom
        return max(float(co2_pct), 0.0)

    def compute_slope_and_std(self, window_s=STATUS_WINDOW_S):
        """Linear-fit slope (%/s) and std-dev of CO2% over the recent window."""
        if len(self.co2_t) < 3:
            return 0.0, 0.0

        t_arr = np.array(self.co2_t)
        v_arr = np.array(self.co2_pct)
        t_now = t_arr[-1]
        mask = t_arr >= (t_now - window_s)
        if mask.sum() < 3:
            mask = np.ones_like(t_arr, dtype=bool)

        t_win = t_arr[mask]
        v_win = v_arr[mask]

        if t_win[-1] == t_win[0]:
            slope = 0.0
        else:
            slope = float(np.polyfit(t_win, v_win, 1)[0])

        std = float(np.std(v_win))
        return slope, std

    def get_pressure_delta_pa(self):
        """Pressure rise above the captured air baseline (sign per Invert checkbox)."""
        if self.baseline_pressure_pa is None:
            return None

        with lock:
            p_now = latest_pressure

        if p_now is None:
            return None

        delta = float(p_now) - self.baseline_pressure_pa
        return -delta if self.pressure_invert else delta

    def classify_breath_status(self, co2_pct, slope, std_dev, pressure_delta):
        """
        Heuristic breath-control classifier for a trombone mouthpiece.
        Primary evidence is CO2 level/slope/stability; pressure is used as a
        faster-responding secondary signal to confirm or lead the CO2 read
        (pressure reacts to airflow essentially instantly, CO2 lags behind
        mixing/diffusion in the bore).
        """
        pressure_blow = (
            pressure_delta is not None and pressure_delta >= self.pressure_blow_threshold_pa
        )

        if co2_pct is None:
            return "No Baseline / Awaiting Capture", "#888888"

        if co2_pct <= CO2_IDLE_PCT:
            if pressure_blow:
                return "Attack - Breath Onset (Pressure Lead)", "#3399ff"
            return "Not Playing (Resting)", "#666666"

        if co2_pct >= CO2_OVERBLOW_PCT and slope >= 0:
            return "High CO2 - Possible Over-blowing", "#ff6600"

        if slope >= SLOPE_RISE_THRESH_PCT_S:
            tag = " + Pressure Confirmed" if pressure_blow else ""
            return f"Attack - Breath Onset{tag}", "#3399ff"

        if slope <= SLOPE_FALL_THRESH_PCT_S:
            return "Release - Breath Ending", "#9966ff"

        if (co2_pct >= CO2_SUSTAIN_MIN_PCT and std_dev <= LEAK_STD_THRESH_PCT
                and abs(slope) < SLOPE_STEADY_BAND_PCT_S):
            if pressure_blow:
                return "Sustained Airflow - Good Support", "#33cc33"
            return "Sustained CO2 - No Pressure Confirmation", "#66aa66"

        if co2_pct > CO2_IDLE_PCT and std_dev > LEAK_STD_THRESH_PCT:
            return "Unstable - Possible Embouchure Leak", "#ffcc00"

        return "Transitional", "#aaaaaa"

    # --- Plot refresh ---
    def update_plot(self):
        if self.paused:
            return

        with lock:
            volts_full = rolling_volts.copy()
            pwm_full = rolling_pwm.copy()
            status = latest_status
            buffer_len = len(rolling_volts)
            t_baro = list(baro_t)
            temp = list(baro_temp)
            pressure = list(baro_pressure)
            temperature_now = latest_temp
            current_packet_count = packet_counter

        # Has a genuinely new packet arrived since the last GUI refresh? LED/
        # Photocurrent history should only advance on real new data -- if it
        # advanced every timer tick regardless, it would keep scrolling ahead
        # during a network stall while Temp/Pressure (which only update on a
        # real packet) sit still, so the two "Time (s)" plot pairs would drift
        # out of sync and Temp/Pressure would appear to fall behind and vanish
        # off the right edge of their shared, linked x-axis.
        has_new_packet = current_packet_count != self.last_seen_packet_count
        self.last_seen_packet_count = current_packet_count

        # ---- 10 ms LED on/off average -> photocurrent (clamped) -> optical power ----
        led_on_avg, led_off_avg = self.compute_led_averages(volts_full, pwm_full)
        i_pd_na = self.compute_photocurrent_na(led_on_avg, led_off_avg, temperature_now)
        p_uw = self.photocurrent_to_optical_power_uw(i_pd_na)

        if has_new_packet and led_on_avg is not None and led_off_avg is not None:
            now = time.time()
            t0 = get_session_start_time(now)
            self.led_t.append(now - t0)
            self.led_on_v.append(led_on_avg)
            self.led_off_v.append(led_off_avg)
            self.photocurrent_na.append(i_pd_na if i_pd_na is not None else float("nan"))

            self.led_on_curve.setData(list(self.led_t), list(self.led_on_v))
            self.led_off_curve.setData(list(self.led_t), list(self.led_off_v))
            self.photocurrent_curve.setData(list(self.led_t), list(self.photocurrent_na))

            i_pd_text = f"{i_pd_na:.2f} nA" if i_pd_na is not None else "rejected (>2nA, clamped)"
            self.led_status_label.setText(
                f"LED On: {led_on_avg:.4f} V | LED Off: {led_off_avg:.4f} V | Photocurrent: {i_pd_text}"
            )

        if p_uw is not None:
            self.power_smooth_window.append(p_uw)

        # ---- Pressure-based secondary blow detection ----
        pressure_delta = self.get_pressure_delta_pa()
        if pressure_delta is not None:
            blow_tag = " [BLOW]" if pressure_delta >= self.pressure_blow_threshold_pa else ""
            self.pressure_status_label.setText(f"Pressure: {pressure_delta:+.2f} Pa vs baseline{blow_tag}")
        else:
            self.pressure_status_label.setText("Pressure: n/a (capture baseline)")

        # ---- CO2 % (needs a captured baseline) ----
        if self.baseline_power_uw is not None and self.power_smooth_window:
            p_smoothed = float(np.mean(self.power_smooth_window))
            co2 = self.compute_co2_percent(p_smoothed)

            if co2 is not None:
                now = time.time()
                if self.co2_start_time is None:
                    self.co2_start_time = now
                self.co2_t.append(now - self.co2_start_time)
                self.co2_pct.append(co2)
                self.co2_curve.setData(list(self.co2_t), list(self.co2_pct))

                slope, std_dev = self.compute_slope_and_std()
                status_text, status_color = self.classify_breath_status(
                    co2, slope, std_dev, pressure_delta
                )

                self.co2_digital_label.setText(f"{co2:5.2f} %")
                self.breath_status_label.setText(status_text)
                self.breath_status_label.setStyleSheet(
                    f"background-color: {status_color}; color: white; padding: 6px; border-radius: 6px;"
                )
        else:
            self.co2_digital_label.setText("--.-- %")

        # ---- ADC / PWM trace, with optional triggering ----
        if len(volts_full):
            triggered = False
            if self.trigger_enabled:
                if self.trigger_source == "ADC Voltage":
                    src_signal = volts_full
                    level = self.trigger_level
                else:
                    src_signal = pwm_full.astype(np.float32)
                    level = 0.5

                result = find_trigger_window(
                    src_signal, self.trace_length, level, self.trigger_edge, self.trigger_pretrigger_frac,
                )
                if result is not None:
                    start, end, trig_idx = result
                    volts = volts_full[start:end]
                    pwm = pwm_full[start:end]
                    pretrigger = int(self.trace_length * self.trigger_pretrigger_frac)
                    x = np.arange(-pretrigger, len(volts) - pretrigger, dtype=np.int32)
                    triggered = True

            if not triggered:
                volts = volts_full[-self.trace_length:]
                pwm = pwm_full[-self.trace_length:]
                x = np.arange(0, len(volts), dtype=np.int32)

            self.adc_curve.setData(x, volts)
            self.pwm_curve.setData(x, pwm.astype(np.float32))
            self.trigger_point_line.setVisible(self.trigger_enabled and triggered)

            self.last_plot_x = x.copy()
            self.last_plot_y = volts.copy()
            self.last_plot_pwm = pwm.copy()

            if self.follow_latest and not (self.trigger_enabled and triggered):
                self.adc_plot.setXRange(0, self.trace_length, padding=0)
            elif self.trigger_enabled and triggered:
                pretrigger = int(self.trace_length * self.trigger_pretrigger_frac)
                self.adc_plot.setXRange(-pretrigger, self.trace_length - pretrigger, padding=0)

            trig_text = " | TRIGGERED" if (self.trigger_enabled and triggered) else (
                " | searching for trigger..." if self.trigger_enabled else ""
            )
            self.adc_plot.setTitle(f"{status} | buffer={buffer_len}{trig_text}")

        if t_baro:
            self.temp_curve.setData(t_baro, temp)

            if self.baseline_pressure_pa is not None:
                pressure_rel = [p - self.baseline_pressure_pa for p in pressure]
                self.pressure_curve.setData(t_baro, pressure_rel)
                self.pressure_plot.setLabel("left", "Pressure (rel. baseline)", units="Pa")
            else:
                self.pressure_curve.setData(t_baro, pressure)
                self.pressure_plot.setLabel("left", "Pressure", units="Pa")

            self.status_label.setText(f"Temp: {temp[-1]:.2f} \u00b0C | Pressure: {pressure[-1]:.1f} Pa")


def main():
    global running

    app = QtWidgets.QApplication(sys.argv)

    reader = threading.Thread(target=udp_thread, daemon=True)
    reader.start()

    window = ScopeWindow()
    window.resize(1700, 1000)
    window.show()

    def cleanup():
        global running
        running = False
        reader.join(timeout=1)

    app.aboutToQuit.connect(cleanup)    

    sys.exit(app.exec())


if __name__ == "__main__":
    main()