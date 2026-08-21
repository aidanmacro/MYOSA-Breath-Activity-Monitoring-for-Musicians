import csv
import socket
import struct
import sys
import threading
import time
from collections import deque

import numpy as np
import pyqtgraph as pg
from PyQt6 import QtCore, QtWidgets

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
ROLLING_BARO_POINTS = 600  # ~10 min at 1 reading/sec
ROLLING_LED_POINTS = 600   # ~10 min of LED on/off + photocurrent history

# --- 10 ms historical averaging window ---
LED_AVG_WINDOW_S = 0.010
LED_AVG_WINDOW_SAMPLES = int(LED_AVG_WINDOW_S * ADC_SAMPLE_RATE_HZ)  # 2000 samples

# --- TIA / analogue front-end model (see Figure 5.2 schematic) ---
TIA_RF_OHMS = 120e3          # Rf1, transimpedance feedback resistor
STAGE2_GAIN = 1 + (240e3 / 1e3)  # Av = 1 + Rf2/Rf3 = 241
TOTAL_TRANSIMPEDANCE = TIA_RF_OHMS * STAGE2_GAIN  # V per A, overall gain

# Photosensitivity temperature coefficient of the InAsSb detector
# (~ -0.1 %/degC around the 4.3 um band, back-illuminated type, per datasheet)
ALPHA_PD_PER_DEGC = -0.001
T0_REF_DEGC = 25.0

# --- Shared state (guarded by `lock`) ---
lock = threading.Lock()
running = True

rolling_volts = np.array([], dtype=np.float32)
rolling_pwm = np.array([], dtype=np.uint8)
latest_status = "Waiting..."

baro_t = deque(maxlen=ROLLING_BARO_POINTS)      # wall-clock seconds
baro_temp = deque(maxlen=ROLLING_BARO_POINTS)   # deg C
baro_pressure = deque(maxlen=ROLLING_BARO_POINTS)  # Pa
baro_start_time = None
latest_temp = None
latest_pressure = None


def checksum_u16(adc_u16):
    return int(np.sum(adc_u16, dtype=np.uint32) & 0xFFFF)


def udp_thread():
    global rolling_volts, rolling_pwm, latest_status
    global baro_start_time, latest_temp, latest_pressure

    # Initialize a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(0.1)

    expected_size = len(MAGIC_BYTES) + HEADER_REST_SIZE + (EXPECTED_SAMPLES * 2)

    try:
        while running:
            try:
                # UDP guarantees we get exactly the message sent by endPacket()
                data, addr = sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break

            # Ignore malformed or fragmented packets
            if len(data) != expected_size:
                continue

            # Verify Magic Bytes
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

            with lock:
                rolling_volts = np.concatenate((rolling_volts, volts))
                rolling_pwm = np.concatenate((rolling_pwm, pwm_state))
                if len(rolling_volts) > ROLLING_ADC_SAMPLES:
                    rolling_volts = rolling_volts[-ROLLING_ADC_SAMPLES:]
                    rolling_pwm = rolling_pwm[-ROLLING_ADC_SAMPLES:]

                latest_status = status

                if baro_start_time is None:
                    baro_start_time = now

                baro_t.append(now - baro_start_time)
                baro_temp.append(float(temperature))
                baro_pressure.append(float(pressure))
                latest_temp = temperature
                latest_pressure = pressure

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

    # Walk backwards from the most recent crossing until one fits in the buffer
    for idx in idxs[::-1]:
        start = idx - pretrigger
        end = start + trace_length
        if start >= 0 and end <= n:
            return start, end, idx

    return None


class ScopeWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESP32 ADC + Barometer Scope")

        self.paused = False
        self.trace_length = 2048
        self.follow_latest = True

        self.last_plot_x = np.array([], dtype=np.float32)
        self.last_plot_y = np.array([], dtype=np.float32)
        self.last_plot_pwm = np.array([], dtype=np.uint8)

        # --- Trigger state ---
        self.trigger_enabled = False
        self.trigger_source = "PWM State"   # or "ADC Voltage"
        self.trigger_edge = "Rising"        # or "Falling"
        self.trigger_level = VREF / 2.0
        self.trigger_pretrigger_frac = 0.25

        # --- LED on/off average + photocurrent history ---
        self.led_start_time = None
        self.led_t = deque(maxlen=ROLLING_LED_POINTS)
        self.led_on_v = deque(maxlen=ROLLING_LED_POINTS)
        self.led_off_v = deque(maxlen=ROLLING_LED_POINTS)
        self.photocurrent_na = deque(maxlen=ROLLING_LED_POINTS)  # nanoamps, temp-compensated

        root = QtWidgets.QVBoxLayout(self)

        # --- Plots ---
        self.plot_widget = pg.GraphicsLayoutWidget()

        self.adc_plot = self.plot_widget.addPlot(
            row=0, col=0, title="Waiting for data..."
        )
        self.adc_plot.setLabel("bottom", "Sample (relative to trigger)")
        self.adc_plot.setLabel("left", "Voltage", units="V")
        self.adc_plot.setYRange(0, VREF)
        self.adc_plot.showGrid(x=True, y=True)
        self.adc_curve = self.adc_plot.plot(pen=pg.mkPen(width=1))

        # Trigger level line (only meaningful for ADC-Voltage source)
        self.trigger_level_line = pg.InfiniteLine(
            pos=self.trigger_level, angle=0,
            pen=pg.mkPen(color="g", width=1, style=QtCore.Qt.PenStyle.DashLine),
            movable=False,
        )
        self.adc_plot.addItem(self.trigger_level_line)
        self.trigger_level_line.setVisible(False)

        # Vertical marker at the trigger instant (t=0)
        self.trigger_point_line = pg.InfiniteLine(
            pos=0, angle=90,
            pen=pg.mkPen(color="g", width=1, style=QtCore.Qt.PenStyle.DashLine),
        )
        self.adc_plot.addItem(self.trigger_point_line)
        self.trigger_point_line.setVisible(False)

        self.pwm_plot = self.plot_widget.addPlot(row=1, col=0, title="PWM Pin State")
        self.pwm_plot.setLabel("bottom", "Sample (relative to trigger)")
        self.pwm_plot.setLabel("left", "State")
        self.pwm_plot.setYRange(-0.2, 1.2, padding=0)
        self.pwm_plot.getAxis("left").setTicks([[(0, "0"), (1, "1")]])
        self.pwm_plot.showGrid(x=True, y=True)
        self.pwm_plot.setXLink(self.adc_plot.getViewBox())
        self.pwm_curve = self.pwm_plot.plot(
            pen=pg.mkPen(width=2, color="y"), stepMode="right"
        )

        self.led_plot = self.plot_widget.addPlot(
            row=2, col=0, title="LED On / Off Photodiode Voltage (10 ms rolling avg)"
        )
        self.led_plot.setLabel("bottom", "Time", units="s")
        self.led_plot.setLabel("left", "Voltage", units="V")
        self.led_plot.showGrid(x=True, y=True)
        self.led_on_curve = self.led_plot.plot(
            pen=pg.mkPen(width=2, color="m"), name="LED On"
        )
        self.led_off_curve = self.led_plot.plot(
            pen=pg.mkPen(width=2, color="w"), name="LED Off"
        )
        self.led_plot.addLegend()

        self.photocurrent_plot = self.plot_widget.addPlot(
            row=3, col=0, title="Photocurrent (temperature-compensated)"
        )
        self.photocurrent_plot.setLabel("bottom", "Time", units="s")
        self.photocurrent_plot.setLabel("left", "Photocurrent", units="nA")
        self.photocurrent_plot.showGrid(x=True, y=True)
        self.photocurrent_plot.setXLink(self.led_plot.getViewBox())
        self.photocurrent_curve = self.photocurrent_plot.plot(
            pen=pg.mkPen(width=2, color="g")
        )

        self.temp_plot = self.plot_widget.addPlot(row=4, col=0, title="Temperature")
        self.temp_plot.setLabel("bottom", "Time", units="s")
        self.temp_plot.setLabel("left", "Temp", units="\u00b0C")
        self.temp_plot.showGrid(x=True, y=True)
        self.temp_curve = self.temp_plot.plot(
            pen=pg.mkPen(width=2, color="r"), symbol="o", symbolSize=4
        )

        self.pressure_plot = self.plot_widget.addPlot(row=5, col=0, title="Pressure")
        self.pressure_plot.setLabel("bottom", "Time", units="s")
        self.pressure_plot.setLabel("left", "Pressure", units="Pa")
        self.pressure_plot.showGrid(x=True, y=True)
        self.pressure_plot.setXLink(self.temp_plot.getViewBox())
        self.pressure_curve = self.pressure_plot.plot(
            pen=pg.mkPen(width=2, color="c"), symbol="o", symbolSize=4
        )

        # Give the ADC trace more vertical room than the other strips
        self.plot_widget.ci.layout.setRowStretchFactor(0, 3)
        self.plot_widget.ci.layout.setRowStretchFactor(1, 1)
        self.plot_widget.ci.layout.setRowStretchFactor(2, 1)
        self.plot_widget.ci.layout.setRowStretchFactor(3, 1)
        self.plot_widget.ci.layout.setRowStretchFactor(4, 1)
        self.plot_widget.ci.layout.setRowStretchFactor(5, 1)

        root.addWidget(self.plot_widget, stretch=1)

        # --- Controls (row 1): pause / export / follow / trace length ---
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

        controls.addWidget(QtWidgets.QLabel("ADC trace length"), 0, 3)

        self.trace_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.trace_slider.setRange(64, ROLLING_ADC_SAMPLES)
        self.trace_slider.setSingleStep(64)
        self.trace_slider.setPageStep(512)
        self.trace_slider.setValue(self.trace_length)
        self.trace_slider.valueChanged.connect(self.update_trace_length)
        controls.addWidget(self.trace_slider, 0, 4)

        self.trace_label = QtWidgets.QLabel(f"{self.trace_length}")
        controls.addWidget(self.trace_label, 0, 5)

        self.status_label = QtWidgets.QLabel("Temp: n/a | Pressure: n/a")
        controls.addWidget(self.status_label, 1, 0, 1, 6)

        self.led_status_label = QtWidgets.QLabel(
            "LED On: n/a | LED Off: n/a | Photocurrent: n/a"
        )
        controls.addWidget(self.led_status_label, 2, 0, 1, 6)

        root.addLayout(controls)

        # --- Trigger controls ---
        trigger_box = QtWidgets.QGroupBox("Trigger")
        trigger_layout = QtWidgets.QHBoxLayout(trigger_box)

        self.trigger_enable_checkbox = QtWidgets.QCheckBox("Enable")
        self.trigger_enable_checkbox.stateChanged.connect(self.update_trigger_enabled)
        trigger_layout.addWidget(self.trigger_enable_checkbox)

        trigger_layout.addWidget(QtWidgets.QLabel("Source"))
        self.trigger_source_combo = QtWidgets.QComboBox()
        self.trigger_source_combo.addItems(["PWM State", "ADC Voltage"])
        self.trigger_source_combo.currentTextChanged.connect(self.update_trigger_source)
        trigger_layout.addWidget(self.trigger_source_combo)

        trigger_layout.addWidget(QtWidgets.QLabel("Edge"))
        self.trigger_edge_combo = QtWidgets.QComboBox()
        self.trigger_edge_combo.addItems(["Rising", "Falling"])
        self.trigger_edge_combo.currentTextChanged.connect(self.update_trigger_edge)
        trigger_layout.addWidget(self.trigger_edge_combo)

        trigger_layout.addWidget(QtWidgets.QLabel("Level (V)"))
        self.trigger_level_spin = QtWidgets.QDoubleSpinBox()
        self.trigger_level_spin.setRange(0.0, VREF)
        self.trigger_level_spin.setSingleStep(0.05)
        self.trigger_level_spin.setValue(self.trigger_level)
        self.trigger_level_spin.setEnabled(False)  # only relevant for ADC Voltage source
        self.trigger_level_spin.valueChanged.connect(self.update_trigger_level)
        trigger_layout.addWidget(self.trigger_level_spin)

        trigger_layout.addStretch(1)
        root.addWidget(trigger_box)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(33)

    # --- UI callbacks ---
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
        self.trigger_level_line.setVisible(
            self.trigger_enabled and self.trigger_source == "ADC Voltage"
        )

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

    def export_csv(self):
        if not self.paused:
            QtWidgets.QMessageBox.warning(
                self, "Export CSV", "Pause the trace before exporting."
            )
            return

        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Export data", "capture.csv", "CSV files (*.csv)"
        )
        if not path:
            return

        base = path[:-4] if path.lower().endswith(".csv") else path

        if len(self.last_plot_x) and len(self.last_plot_y):
            with open(base + "_adc.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["sample", "time_s", "voltage_v", "pwm_state"])
                for i, (sample, voltage) in enumerate(
                    zip(self.last_plot_x, self.last_plot_y)
                ):
                    pwm_val = (
                        int(self.last_plot_pwm[i])
                        if i < len(self.last_plot_pwm)
                        else ""
                    )
                    writer.writerow([
                        int(sample),
                        float(sample) / float(ADC_SAMPLE_RATE_HZ),
                        float(voltage),
                        pwm_val,
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
                writer.writerow([
                    "time_s", "led_on_v", "led_off_v", "photocurrent_na"
                ])
                for ti, von, voff, ipd in zip(
                    self.led_t, self.led_on_v, self.led_off_v, self.photocurrent_na
                ):
                    writer.writerow([ti, von, voff, ipd])

    # --- LED on/off averaging + photocurrent ---
    def compute_led_averages(self, volts_full, pwm_full):
        """
        Average the ADC voltage over the last LED_AVG_WINDOW_SAMPLES samples
        (10 ms @ ADC_SAMPLE_RATE_HZ), split by PWM (LED) state.
        Returns (led_on_avg, led_off_avg) or (None, None) if a state has no
        samples in the window yet.
        """
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
        Photocurrent from the TIA + 2nd stage gain, using the LED-off sample
        as the instantaneous baseline (this cancels Vref/offset/bias-current
        drift terms common to both states), then temperature-compensated
        back to a 25 degC-equivalent reading using the photodiode's
        photosensitivity temperature coefficient.
        """
        if led_on_avg is None or led_off_avg is None:
            return None

        delta_v = led_on_avg - led_off_avg
        i_pd_raw = delta_v / TOTAL_TRANSIMPEDANCE  # amps

        if temperature_c is not None:
            denom = 1.0 + ALPHA_PD_PER_DEGC * (temperature_c - T0_REF_DEGC)
            if denom != 0:
                i_pd_raw = i_pd_raw / denom

        return i_pd_raw * 1e9  # nanoamps

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

        # ---- 10 ms LED on/off average + photocurrent (uses full chronological buffer) ----
        led_on_avg, led_off_avg = self.compute_led_averages(volts_full, pwm_full)
        i_pd_na = self.compute_photocurrent_na(led_on_avg, led_off_avg, temperature_now)

        if led_on_avg is not None and led_off_avg is not None:
            now = time.time()
            if self.led_start_time is None:
                self.led_start_time = now
            self.led_t.append(now - self.led_start_time)
            self.led_on_v.append(led_on_avg)
            self.led_off_v.append(led_off_avg)
            self.photocurrent_na.append(i_pd_na if i_pd_na is not None else float("nan"))

            self.led_on_curve.setData(list(self.led_t), list(self.led_on_v))
            self.led_off_curve.setData(list(self.led_t), list(self.led_off_v))
            self.photocurrent_curve.setData(list(self.led_t), list(self.photocurrent_na))

            i_pd_text = f"{i_pd_na:.2f} nA" if i_pd_na is not None else "n/a"
            self.led_status_label.setText(
                f"LED On: {led_on_avg:.4f} V | LED Off: {led_off_avg:.4f} V | "
                f"Photocurrent: {i_pd_text}"
            )

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
                    src_signal, self.trace_length, level, self.trigger_edge,
                    self.trigger_pretrigger_frac,
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
                self.adc_plot.setXRange(
                    -pretrigger, self.trace_length - pretrigger, padding=0
                )

            trig_text = " | TRIGGERED" if (self.trigger_enabled and triggered) else (
                " | searching for trigger..." if self.trigger_enabled else ""
            )
            self.adc_plot.setTitle(f"{status} | buffer={buffer_len}{trig_text}")

        if t_baro:
            self.temp_curve.setData(t_baro, temp)
            self.pressure_curve.setData(t_baro, pressure)

            self.status_label.setText(
                f"Temp: {temp[-1]:.2f} \u00b0C | Pressure: {pressure[-1]:.1f} Pa"
            )


def main():
    global running

    app = QtWidgets.QApplication(sys.argv)

    # FIXED: Thread target changed from serial_thread to udp_thread
    reader = threading.Thread(target=udp_thread, daemon=True)
    reader.start()

    window = ScopeWindow()
    window.resize(1200, 1200)
    window.show()

    def cleanup():
        global running
        running = False
        reader.join(timeout=1)

    app.aboutToQuit.connect(cleanup)

    sys.exit(app.exec())


if __name__ == "__main__":
    main()