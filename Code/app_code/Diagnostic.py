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

# ---------------------------------------------------------
# Network & Packet Configuration
# ---------------------------------------------------------
# We listen on all network interfaces to catch the incoming UDP stream.
UDP_IP = "0.0.0.0"  
UDP_PORT = 12345

# Our hardware sends 512 samples per packet. We use a magic byte sequence 
# to ensure we don't try to parse stray network garbage.
EXPECTED_SAMPLES = 512
MAGIC_BYTES = b"OCIP!CDA"

# The struct format for our packet header: 
# sequence (I), dropped (I), samples (H), checksum (H), temp (f), pressure (f).
HEADER_REST_FMT = "<I I H H f f"
HEADER_REST_SIZE = struct.calcsize(HEADER_REST_FMT)

# ---------------------------------------------------------
# Hardware Specifications & Scaling
# ---------------------------------------------------------
VREF = 3.3
ADC_MAX = 4095.0
ADC_SAMPLE_RATE_HZ = 200000

# Rolling buffers to keep the GUI from consuming infinite memory.
# Sizes are chosen based on their respective sample rates to give us a good historical window.
ROLLING_ADC_SAMPLES = 8192
ROLLING_BARO_POINTS = 600     # Roughly 10 minutes at 1Hz
ROLLING_LED_POINTS = 600      
ROLLING_CO2_POINTS = 4000     # Roughly 2 minutes of CO2 history at ~30Hz

# We use a 10 ms window to average the LED states and smooth out high-frequency noise.
LED_AVG_WINDOW_S = 0.10
LED_AVG_WINDOW_SAMPLES = int(LED_AVG_WINDOW_S * ADC_SAMPLE_RATE_HZ)

# Transimpedance Amplifier (TIA) model. 
# This dictates how we convert the raw voltage back into physical current.
TIA_RF_OHMS = 120e3               
STAGE2_GAIN = 1 + (240e3 / 1e3)   # Av = 241
TOTAL_TRANSIMPEDANCE = TIA_RF_OHMS * STAGE2_GAIN  

# InAsSb detector temperature compensation (approx -0.1 %/degC).
ALPHA_PD_PER_DEGC = -0.001
T0_REF_DEGC = 25.0
DETECTOR_RESPONSIVITY_A_PER_W = 4.5e-3  

# Sanity check: Based on the physical geometry of our optical path and the LED's 400mA 
# drive pulses, getting a photocurrent reading above 2 nA is physically impossible. 
# We clamp it here to prevent glitches from blowing up our CO2 math.
MAX_PHYSICAL_PHOTOCURRENT_NA = 2.0

# ---------------------------------------------------------
# Calibration & Breath Detection Thresholds
# ---------------------------------------------------------
DEFAULT_PATH_LENGTH_CM = 3
DEFAULT_ABS_COEFF_PER_PCT_CM = 0.10

# These heuristics determine what counts as "blowing" into the trombone mouthpiece.
# They might need tweaking depending on the specific player or physical enclosure.
CO2_IDLE_PCT = 0.20
CO2_SUSTAIN_MIN_PCT = 1.00
CO2_OVERBLOW_PCT = 5.00
SLOPE_RISE_THRESH_PCT_S = 3.0
SLOPE_FALL_THRESH_PCT_S = -3.0
SLOPE_STEADY_BAND_PCT_S = 1.0
LEAK_STD_THRESH_PCT = 0.40
STATUS_WINDOW_S = 0.75

# A sudden spike in pressure tells us the breath started *before* the CO2 has time 
# to diffuse into the sensor path. It's our early warning system.
DEFAULT_PRESSURE_BLOW_THRESHOLD_PA = 0.05

# ---------------------------------------------------------
# Shared Global State
# ---------------------------------------------------------
# The lock ensures our background UDP thread and the GUI don't try to read/write 
# these variables at the exact same millisecond.
lock = threading.Lock()
running = True

rolling_volts = np.array([], dtype=np.float32)
rolling_pwm = np.array([], dtype=np.uint8)
latest_status = "Waiting..."
packet_counter = 0  

baro_t = deque(maxlen=ROLLING_BARO_POINTS)      
baro_temp = deque(maxlen=ROLLING_BARO_POINTS)   
baro_pressure = deque(maxlen=ROLLING_BARO_POINTS)  
latest_temp = None
latest_pressure = None

# We anchor all plots to a single 'time zero' so that panning and zooming 
# across different graphs keeps everything perfectly synced up.
session_start_time = None

def get_session_start_time(now):
    """Fetches the shared t=0 reference, creating it if this is the first data point."""
    global session_start_time
    with lock:
        if session_start_time is None:
            session_start_time = now
        return session_start_time

def checksum_u16(adc_u16):
    """Simple 16-bit summation checksum to catch corrupt packets."""
    return int(np.sum(adc_u16, dtype=np.uint32) & 0xFFFF)

def udp_thread():
    """
    Background worker that constantly listens for UDP packets, validates them,
    and safely pushes the data into our rolling buffers.
    """
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

            # Drop packets that don't match our exact expected size or magic header.
            if len(data) != expected_size or data[:len(MAGIC_BYTES)] != MAGIC_BYTES:
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

            # Extract the PWM state (bit 15) and the actual 12-bit ADC value.
            pwm_state = ((adc_u16 >> 15) & 0x1).astype(np.uint8)
            adc_u16 = adc_u16 & 0x0FFF
            volts = adc_u16.astype(np.float32) * (VREF / ADC_MAX)

            now = time.time()
            t0 = get_session_start_time(now)
            
            # Safely update the globals so the GUI can pick them up on its next tick.
            with lock:
                rolling_volts = np.concatenate((rolling_volts, volts))
                rolling_pwm = np.concatenate((rolling_pwm, pwm_state))
                
                # Keep the high-speed buffers from growing indefinitely
                if len(rolling_volts) > ROLLING_ADC_SAMPLES:
                    rolling_volts = rolling_volts[-ROLLING_ADC_SAMPLES:]
                    rolling_pwm = rolling_pwm[-ROLLING_ADC_SAMPLES:]

                latest_status = f"min={volts.min():.3f} V | max={volts.max():.3f} V | Dropped={_dropped}"
                
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
    Acts like a real hardware oscilloscope trigger. We scan backwards to find the 
    most recent edge crossing so the plot stays stable on screen.
    """
    n = len(signal)
    if n < 2 or trace_length < 2:
        return None

    pretrigger = int(trace_length * pretrigger_frac)

    if edge == "Rising":
        idxs = np.where((signal[:-1] < level) & (signal[1:] >= level))[0] + 1
    else:  
        idxs = np.where((signal[:-1] >= level) & (signal[1:] < level))[0] + 1

    for idx in idxs[::-1]:
        start = idx - pretrigger
        end = start + trace_length
        if start >= 0 and end <= n:
            return start, end, idx

    return None

class ScopeWindow(QtWidgets.QWidget):
    """The main GUI application for visualizing sensor data and inferring breath states."""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Trombone Mouthpiece CO2 / Breath Monitor")

        self.paused = False
        self.trace_length = 2048
        self.follow_latest = True

        self.last_plot_x = np.array([], dtype=np.float32)
        self.last_plot_y = np.array([], dtype=np.float32)
        self.last_plot_pwm = np.array([], dtype=np.uint8)

        # Trigger state defaults
        self.trigger_enabled = False
        self.trigger_source = "PWM State"
        self.trigger_edge = "Rising"
        self.trigger_level = VREF / 2.0
        self.trigger_pretrigger_frac = 0.25

        # Optical history tracking
        self.last_seen_packet_count = -1  
        self.led_t = deque(maxlen=ROLLING_LED_POINTS)
        self.led_on_v = deque(maxlen=ROLLING_LED_POINTS)
        self.led_off_v = deque(maxlen=ROLLING_LED_POINTS)
        self.photocurrent_na = deque(maxlen=ROLLING_LED_POINTS)  

        self.power_smooth_window = deque(maxlen=5)

        # Baseline and CO2 tracking
        self.baseline_power_uw = None
        self.baseline_pressure_pa = None
        self.co2_path_length_cm = DEFAULT_PATH_LENGTH_CM
        self.co2_abs_coeff = DEFAULT_ABS_COEFF_PER_PCT_CM
        self.co2_start_time = None
        self.co2_t = deque(maxlen=ROLLING_CO2_POINTS)
        self.co2_pct = deque(maxlen=ROLLING_CO2_POINTS)

        self.pressure_blow_threshold_pa = DEFAULT_PRESSURE_BLOW_THRESHOLD_PA
        self.pressure_invert = False

        self._setup_ui()

        # The timer acts as our main loop, pulling data roughly at 30 FPS.
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(33)

    def _setup_ui(self):
        """Builds out the layout: main CO2 focus on the left, diagnostics on the right."""
        root = QtWidgets.QHBoxLayout(self)
        left_col = QtWidgets.QVBoxLayout()
        right_col = QtWidgets.QVBoxLayout()
        root.addLayout(left_col, stretch=6)
        root.addLayout(right_col, stretch=5)

        # --- LEFT COLUMN: Primary Data ---
        co2_group = QtWidgets.QGroupBox("CO2 Monitor (NDIR, 4.26 \u00b5m band)")
        co2_layout = QtWidgets.QVBoxLayout(co2_group)
        top_row = QtWidgets.QHBoxLayout()

        self.co2_digital_label = QtWidgets.QLabel("--.-- %")
        self.co2_digital_label.setFont(QtGui.QFont("Consolas", 48, QtGui.QFont.Weight.Bold))
        self.co2_digital_label.setStyleSheet("background-color: black; color: #33ff33; padding: 8px; border-radius: 6px;")
        self.co2_digital_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.co2_digital_label.setMinimumWidth(260)
        top_row.addWidget(self.co2_digital_label)

        status_col = QtWidgets.QVBoxLayout()
        self.breath_status_label = QtWidgets.QLabel("No Baseline / Awaiting Capture")
        self.breath_status_label.setFont(QtGui.QFont("Arial", 16, QtGui.QFont.Weight.Bold))
        self.breath_status_label.setStyleSheet("background-color: #444444; color: white; padding: 6px; border-radius: 6px;")
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

        # Calibration controls
        calib_row1 = QtWidgets.QHBoxLayout()
        calib_row1.addWidget(QtWidgets.QLabel("Path length (cm)"))
        self.path_length_spin = QtWidgets.QDoubleSpinBox()
        self.path_length_spin.setRange(0.1, 20.0)
        self.path_length_spin.setValue(self.co2_path_length_cm)
        self.path_length_spin.valueChanged.connect(self.update_path_length)
        calib_row1.addWidget(self.path_length_spin)

        calib_row1.addWidget(QtWidgets.QLabel("Abs. coeff (per %CO2\u00b7cm)"))
        self.abs_coeff_spin = QtWidgets.QDoubleSpinBox()
        self.abs_coeff_spin.setRange(0.01, 10.0)
        self.abs_coeff_spin.setValue(self.co2_abs_coeff)
        self.abs_coeff_spin.valueChanged.connect(self.update_abs_coeff)
        calib_row1.addWidget(self.abs_coeff_spin)
        calib_row1.addStretch(1)
        status_col.addLayout(calib_row1)

        calib_row2 = QtWidgets.QHBoxLayout()
        calib_row2.addWidget(QtWidgets.QLabel("Pressure blow thresh. (Pa)"))
        self.pressure_thresh_spin = QtWidgets.QDoubleSpinBox()
        self.pressure_thresh_spin.setRange(0.1, 200.0)
        self.pressure_thresh_spin.setValue(self.pressure_blow_threshold_pa)
        self.pressure_thresh_spin.valueChanged.connect(self.update_pressure_threshold)
        calib_row2.addWidget(self.pressure_thresh_spin)

        self.pressure_invert_checkbox = QtWidgets.QCheckBox("Invert sign")
        self.pressure_invert_checkbox.stateChanged.connect(self.update_pressure_invert)
        calib_row2.addWidget(self.pressure_invert_checkbox)
        calib_row2.addStretch(1)
        status_col.addLayout(calib_row2)

        top_row.addLayout(status_col, stretch=1)
        co2_layout.addLayout(top_row)

        # CO2 Graph
        self.co2_plot_widget = pg.PlotWidget(title="CO2 % vs Time (since baseline capture)")
        self.co2_plot_widget.setLabel("bottom", "Time", units="s")
        self.co2_plot_widget.setLabel("left", "CO2", units="%")
        self.co2_plot_widget.showGrid(x=True, y=True)
        self.co2_curve = self.co2_plot_widget.plot(pen=pg.mkPen(width=2, color="#33ff33"))
        
        # Guide lines for breath statuses
        self.co2_idle_line = pg.InfiniteLine(pos=CO2_IDLE_PCT, angle=0, pen=pg.mkPen(color="#888888", width=1, style=QtCore.Qt.PenStyle.DashLine))
        self.co2_sustain_line = pg.InfiniteLine(pos=CO2_SUSTAIN_MIN_PCT, angle=0, pen=pg.mkPen(color="#3399ff", width=1, style=QtCore.Qt.PenStyle.DashLine))
        self.co2_plot_widget.addItem(self.co2_idle_line)
        self.co2_plot_widget.addItem(self.co2_sustain_line)
        self.co2_plot_widget.setMinimumHeight(420)
        co2_layout.addWidget(self.co2_plot_widget, stretch=1)

        left_col.addWidget(co2_group, stretch=1)

        # --- RIGHT COLUMN: Diagnostic Scopes ---
        self.plot_widget = pg.GraphicsLayoutWidget()

        # 1. Raw ADC
        self.adc_plot = self.plot_widget.addPlot(row=0, col=0, title="Waiting for data...")
        self.adc_plot.setLabel("bottom", "Sample (rel. trigger)")
        self.adc_plot.setLabel("left", "Voltage", units="V")
        self.adc_plot.setYRange(0, VREF)
        self.adc_plot.showGrid(x=True, y=True)
        self.adc_curve = self.adc_plot.plot(pen=pg.mkPen(width=1))

        self.trigger_level_line = pg.InfiniteLine(pos=self.trigger_level, angle=0, pen=pg.mkPen(color="g", width=1, style=QtCore.Qt.PenStyle.DashLine), movable=False)
        self.adc_plot.addItem(self.trigger_level_line)
        self.trigger_level_line.setVisible(False)

        self.trigger_point_line = pg.InfiniteLine(pos=0, angle=90, pen=pg.mkPen(color="g", width=1, style=QtCore.Qt.PenStyle.DashLine))
        self.adc_plot.addItem(self.trigger_point_line)
        self.trigger_point_line.setVisible(False)

        # 2. PWM Pin State
        self.pwm_plot = self.plot_widget.addPlot(row=1, col=0, title="PWM Pin State")
        self.pwm_plot.setLabel("bottom", "Sample (rel. trigger)")
        self.pwm_plot.setLabel("left", "State")
        self.pwm_plot.setYRange(-0.2, 1.2, padding=0)
        self.pwm_plot.getAxis("left").setTicks([[(0, "0"), (1, "1")]])
        self.pwm_plot.showGrid(x=True, y=True)
        self.pwm_plot.setXLink(self.adc_plot.getViewBox())
        self.pwm_curve = self.pwm_plot.plot(pen=pg.mkPen(width=2, color="y"), stepMode="right")

        # 3. LED Voltage Avg
        self.led_plot = self.plot_widget.addPlot(row=2, col=0, title="LED On/Off Photodiode Voltage (10 ms avg)")
        self.led_plot.setLabel("bottom", "Time", units="s")
        self.led_plot.setLabel("left", "Voltage", units="V")
        self.led_plot.showGrid(x=True, y=True)
        self.led_on_curve = self.led_plot.plot(pen=pg.mkPen(width=2, color="m"), name="LED On")
        self.led_off_curve = self.led_plot.plot(pen=pg.mkPen(width=2, color="w"), name="LED Off")
        self.led_plot.addLegend()

        # 4. Photocurrent
        self.photocurrent_plot = self.plot_widget.addPlot(row=3, col=0, title="Photocurrent (clamped to \u00b12 nA)")
        self.photocurrent_plot.setLabel("bottom", "Time", units="s")
        self.photocurrent_plot.setLabel("left", "Photocurrent", units="nA")
        self.photocurrent_plot.showGrid(x=True, y=True)
        self.photocurrent_plot.setXLink(self.led_plot.getViewBox())
        self.photocurrent_curve = self.photocurrent_plot.plot(pen=pg.mkPen(width=2, color="g"))

        # 5. Temperature
        self.temp_plot = self.plot_widget.addPlot(row=4, col=0, title="Temperature")
        self.temp_plot.setXLink(self.led_plot.getViewBox())
        self.temp_plot.setLabel("bottom", "Time", units="s")
        self.temp_plot.setLabel("left", "Temp", units="\u00b0C")
        self.temp_plot.showGrid(x=True, y=True)
        self.temp_curve = self.temp_plot.plot(pen=pg.mkPen(width=2, color="r"), symbol="o", symbolSize=4)

        # 6. Pressure
        self.pressure_plot = self.plot_widget.addPlot(row=5, col=0, title="Pressure (with blow-detect baseline)")
        self.pressure_plot.setLabel("bottom", "Time", units="s")
        self.pressure_plot.setLabel("left", "Pressure", units="Pa")
        self.pressure_plot.showGrid(x=True, y=True)
        self.pressure_plot.setXLink(self.temp_plot.getViewBox())
        self.pressure_curve = self.pressure_plot.plot(pen=pg.mkPen(width=2, color="c"), symbol="o", symbolSize=4)
        
        self.pressure_baseline_line = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen(color="#888888", width=1, style=QtCore.Qt.PenStyle.DashLine))
        self.pressure_plot.addItem(self.pressure_baseline_line)
        self.pressure_baseline_line.setVisible(False)

        # Stretch equally
        for r in range(6):
            self.plot_widget.ci.layout.setRowStretchFactor(r, 1)

        right_col.addWidget(self.plot_widget, stretch=1)

        # Setup interaction controls
        self._setup_controls(right_col)

    def _setup_controls(self, layout):
        """Adds buttons and sliders for managing trace captures."""
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

        layout.addLayout(controls)

        # Trigger settings
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

        layout.addWidget(trigger_box)

    # --- UI Callbacks ---
    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_button.setText("Run" if self.paused else "Pause")

    def update_follow_latest(self, state):
        self.follow_latest = state == QtCore.Qt.CheckState.Checked.value

    def update_trace_length(self, value):
        self.trace_length = max(64, int(value))
        self.trace_label.setText(f"{self.trace_length}")

    def update_trigger_enabled(self, state):
        self.trigger_enabled = state == QtCore.Qt.CheckState.Checked.value
        self.trigger_point_line.setVisible(self.trigger_enabled)
        self.trigger_level_line.setVisible(self.trigger_enabled and self.trigger_source == "ADC Voltage")

    def update_trigger_source(self, text):
        self.trigger_source = text
        is_adc = (text == "ADC Voltage")
        self.trigger_level_spin.setEnabled(is_adc)
        self.trigger_level_line.setVisible(self.trigger_enabled and is_adc)

    def update_trigger_edge(self, text):
        self.trigger_edge = text

    def update_trigger_level(self, value):
        self.trigger_level = float(value)
        self.trigger_level_line.setPos(self.trigger_level)

    def update_path_length(self, value):
        self.co2_path_length_cm = float(value)

    def update_abs_coeff(self, value):
        self.co2_abs_coeff = float(value)

    def update_pressure_threshold(self, value):
        self.pressure_blow_threshold_pa = float(value)

    def update_pressure_invert(self, state):
        self.pressure_invert = state == QtCore.Qt.CheckState.Checked.value

    def capture_baseline(self):
        """Records ambient conditions so we can calculate relative CO2 and pressure spikes."""
        if not self.power_smooth_window:
            QtWidgets.QMessageBox.warning(
                self, "Capture Baseline",
                "No photocurrent data yet. Wait for the sensor to stream."
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

        # Reset main CO2 display relative to this new snapshot
        self.co2_start_time = time.time()
        self.co2_t.clear()
        self.co2_pct.clear()
        self.co2_curve.setData([], [])

    def export_csv(self):
        """Dumps all captured buffers to CSV for offline analysis."""
        if not self.paused:
            QtWidgets.QMessageBox.warning(self, "Export CSV", "Pause the trace before exporting.")
            return

        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Export data", "capture.csv", "CSV files (*.csv)")
        if not path:
            return

        base = path[:-4] if path.lower().endswith(".csv") else path

        # Export ADC
        if len(self.last_plot_x) and len(self.last_plot_y):
            with open(base + "_adc.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["sample", "time_s", "voltage_v", "pwm_state"])
                for i, (sample, voltage) in enumerate(zip(self.last_plot_x, self.last_plot_y)):
                    pwm_val = int(self.last_plot_pwm[i]) if i < len(self.last_plot_pwm) else ""
                    writer.writerow([int(sample), float(sample) / ADC_SAMPLE_RATE_HZ, float(voltage), pwm_val])

        # Export Barometric Data
        with lock:
            t, temp, pressure = list(baro_t), list(baro_temp), list(baro_pressure)
        if t:
            with open(base + "_baro.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["time_s", "temperature_c", "pressure_pa"])
                for ti, tempi, pi in zip(t, temp, pressure):
                    writer.writerow([ti, tempi, pi])

        # Export Optical Data
        if self.led_t:
            with open(base + "_photocurrent.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["time_s", "led_on_v", "led_off_v", "photocurrent_na"])
                for ti, von, voff, ipd in zip(self.led_t, self.led_on_v, self.led_off_v, self.photocurrent_na):
                    writer.writerow([ti, von, voff, ipd])

        # Export CO2 Data
        if self.co2_t:
            with open(base + "_co2.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["time_s", "co2_pct"])
                for ti, c in zip(self.co2_t, self.co2_pct):
                    writer.writerow([ti, c])

    # --- Calculation Helpers ---
    def compute_led_averages(self, volts_full, pwm_full):
        """Splits the ADC buffer by LED state to find the on/off averages."""
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
        Subtracts the LED-off baseline from the LED-on signal to cancel out ambient 
        noise, converts via TIA gain, and normalizes for temperature.
        """
        if led_on_avg is None or led_off_avg is None:
            return None

        delta_v = led_on_avg - led_off_avg
        i_pd_raw = delta_v / TOTAL_TRANSIMPEDANCE 

        if temperature_c is not None:
            denom = 1.0 + ALPHA_PD_PER_DEGC * (temperature_c - T0_REF_DEGC)
            if denom != 0:
                i_pd_raw = i_pd_raw / denom

        i_pd_na = i_pd_raw * 1e9

        if abs(i_pd_na) > MAX_PHYSICAL_PHOTOCURRENT_NA:
            return None  # Discard if it violates hardware physics

        return i_pd_na

    def photocurrent_to_optical_power_uw(self, i_pd_na):
        """Converts expected photocurrent back into raw incident optical power."""
        if i_pd_na is None:
            return None
        i_pd_a = i_pd_na * 1e-9
        p_w = i_pd_a / DETECTOR_RESPONSIVITY_A_PER_W
        return p_w * 1e6

    def compute_co2_percent(self, p_measured_uw):
        """Applies the Beer-Lambert law using the captured baseline."""
        if not p_measured_uw or not self.baseline_power_uw or p_measured_uw <= 0 or self.baseline_power_uw <= 0:
            return None

        transmittance = p_measured_uw / self.baseline_power_uw
        if transmittance <= 0:
            return None

        absorbance = -np.log(transmittance)
        denom = self.co2_abs_coeff * self.co2_path_length_cm
        if denom == 0:
            return None

        return max(float(absorbance / denom), 0.0)

    def compute_slope_and_std(self, window_s=STATUS_WINDOW_S):
        """Provides a quick linear fit to gauge if CO2 is rising or stable."""
        if len(self.co2_t) < 3:
            return 0.0, 0.0

        t_arr = np.array(self.co2_t)
        v_arr = np.array(self.co2_pct)
        mask = t_arr >= (t_arr[-1] - window_s)
        
        if mask.sum() < 3:
            mask = np.ones_like(t_arr, dtype=bool)

        t_win, v_win = t_arr[mask], v_arr[mask]
        slope = 0.0 if t_win[-1] == t_win[0] else float(np.polyfit(t_win, v_win, 1)[0])
        return slope, float(np.std(v_win))

    def get_pressure_delta_pa(self):
        """Calculates current pressure offset from baseline."""
        if self.baseline_pressure_pa is None:
            return None
        with lock:
            p_now = latest_pressure
        if p_now is None:
            return None
        
        delta = float(p_now) - self.baseline_pressure_pa
        return -delta if self.pressure_invert else delta

    def classify_breath_status(self, co2_pct, slope, std_dev, pressure_delta):
        """Combines CO2 trends and pressure spikes to infer embouchure state."""
        pressure_blow = (pressure_delta is not None and pressure_delta >= self.pressure_blow_threshold_pa)

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

        if (co2_pct >= CO2_SUSTAIN_MIN_PCT and std_dev <= LEAK_STD_THRESH_PCT and abs(slope) < SLOPE_STEADY_BAND_PCT_S):
            if pressure_blow:
                return "Sustained Airflow - Good Support", "#33cc33"
            return "Sustained CO2 - No Pressure Confirmation", "#66aa66"

        if co2_pct > CO2_IDLE_PCT and std_dev > LEAK_STD_THRESH_PCT:
            return "Unstable - Possible Embouchure Leak", "#ffcc00"

        return "Transitional", "#aaaaaa"

    # --- Plot Rendering ---
    def update_plot(self):
        """Timer callback that reads from globals and updates all pyqtgraph widgets."""
        if self.paused:
            return

        with lock:
            volts_full = rolling_volts.copy()
            pwm_full = rolling_pwm.copy()
            status = latest_status
            buffer_len = len(rolling_volts)
            t_baro, temp, pressure = list(baro_t), list(baro_temp), list(baro_pressure)
            temperature_now = latest_temp
            current_packet_count = packet_counter

        # Ensure we only advance optical traces if new UDP data actually arrived
        has_new_packet = current_packet_count != self.last_seen_packet_count
        self.last_seen_packet_count = current_packet_count

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
            self.led_status_label.setText(f"LED On: {led_on_avg:.4f} V | LED Off: {led_off_avg:.4f} V | Photocurrent: {i_pd_text}")

        if p_uw is not None:
            self.power_smooth_window.append(p_uw)

        # Handle Pressure Status
        pressure_delta = self.get_pressure_delta_pa()
        if pressure_delta is not None:
            blow_tag = " [BLOW]" if pressure_delta >= self.pressure_blow_threshold_pa else ""
            self.pressure_status_label.setText(f"Pressure: {pressure_delta:+.2f} Pa vs baseline{blow_tag}")
        else:
            self.pressure_status_label.setText("Pressure: n/a (capture baseline)")

        # Handle CO2 Computations & Display
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
                status_text, status_color = self.classify_breath_status(co2, slope, std_dev, pressure_delta)

                self.co2_digital_label.setText(f"{co2:5.2f} %")
                self.breath_status_label.setText(status_text)
                self.breath_status_label.setStyleSheet(f"background-color: {status_color}; color: white; padding: 6px; border-radius: 6px;")
        else:
            self.co2_digital_label.setText("--.-- %")

        # Update Scope traces
        if len(volts_full):
            triggered = False
            if self.trigger_enabled:
                src_signal = volts_full if self.trigger_source == "ADC Voltage" else pwm_full.astype(np.float32)
                level = self.trigger_level if self.trigger_source == "ADC Voltage" else 0.5
                
                result = find_trigger_window(src_signal, self.trace_length, level, self.trigger_edge, self.trigger_pretrigger_frac)
                if result is not None:
                    start, end, trig_idx = result
                    volts, pwm = volts_full[start:end], pwm_full[start:end]
                    pretrigger = int(self.trace_length * self.trigger_pretrigger_frac)
                    x = np.arange(-pretrigger, len(volts) - pretrigger, dtype=np.int32)
                    triggered = True

            if not triggered:
                volts, pwm = volts_full[-self.trace_length:], pwm_full[-self.trace_length:]
                x = np.arange(0, len(volts), dtype=np.int32)

            self.adc_curve.setData(x, volts)
            self.pwm_curve.setData(x, pwm.astype(np.float32))
            self.trigger_point_line.setVisible(self.trigger_enabled and triggered)

            self.last_plot_x, self.last_plot_y, self.last_plot_pwm = x.copy(), volts.copy(), pwm.copy()

            # Autopan camera if tracking latest point
            if self.follow_latest and not (self.trigger_enabled and triggered):
                self.adc_plot.setXRange(0, self.trace_length, padding=0)
            elif self.trigger_enabled and triggered:
                pretrigger = int(self.trace_length * self.trigger_pretrigger_frac)
                self.adc_plot.setXRange(-pretrigger, self.trace_length - pretrigger, padding=0)

            trig_text = " | TRIGGERED" if (self.trigger_enabled and triggered) else (" | searching for trigger..." if self.trigger_enabled else "")
            self.adc_plot.setTitle(f"{status} | buffer={buffer_len}{trig_text}")

        # Update Barometric visuals
        if t_baro:
            self.temp_curve.setData(t_baro, temp)
            if self.baseline_pressure_pa is not None:
                self.pressure_curve.setData(t_baro, [p - self.baseline_pressure_pa for p in pressure])
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