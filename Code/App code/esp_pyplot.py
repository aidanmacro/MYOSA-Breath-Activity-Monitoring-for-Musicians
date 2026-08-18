import sys
import struct
import threading
import csv

import serial
import numpy as np
import pyqtgraph as pg
from PyQt6 import QtWidgets, QtCore

PORT = "COM8"
BAUD = 2000000
EXPECTED_SAMPLES = 512

ROLLING_BUFFER_SAMPLES = 8192

VREF = 3.3
ADC_MAX = 4095.0
ADC_SAMPLE_RATE_HZ = 200000

MAGIC_BYTES = b"OCIP!CDA"

HEADER_REST_FMT = "<I I H H"
HEADER_REST_SIZE = struct.calcsize(HEADER_REST_FMT)

latest_status = "Waiting..."
rolling_volts = np.array([], dtype=np.float32)

lock = threading.Lock()
running = True


def checksum_u16(adc_u16):
    return int(np.sum(adc_u16, dtype=np.uint32) & 0xFFFF)


def read_exact(ser, n):
    buf = bytearray(n)
    view = memoryview(buf)
    got = 0

    while running and got < n:
        r = ser.readinto(view[got:])
        if r:
            got += r

    return bytes(buf) if got == n else None


def wait_for_magic(ser):
    window = bytearray()

    while running:
        b = ser.read(1)

        if not b:
            continue

        window += b

        if len(window) > len(MAGIC_BYTES):
            del window[0]

        if bytes(window) == MAGIC_BYTES:
            return True

    return False


def serial_thread():
    global rolling_volts, latest_status

    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    ser.dtr = False
    ser.rts = False
    ser.reset_input_buffer()

    try:
        while running:
            if not wait_for_magic(ser):
                break

            rest = read_exact(ser, HEADER_REST_SIZE)

            if rest is None:
                break

            _sequence, _dropped, samples, checksum = struct.unpack(
                HEADER_REST_FMT,
                rest
            )

            if samples != EXPECTED_SAMPLES:
                continue

            raw = read_exact(ser, samples * 2)

            if raw is None:
                break

            adc_u16 = np.frombuffer(raw, dtype="<u2").copy()

            #if adc_u16.max() > 4095:
            #    continue

            if checksum_u16(adc_u16) != checksum:
                continue
            # 2. Mask the upper 4 bits to guarantee values stay in the 0-4095 range
            adc_u16 = adc_u16 & 0x0FFF

            volts = adc_u16.astype(np.float32) * (VREF / ADC_MAX)
            
            status = (
                f"min={volts.min():.3f} V | "
                f"max={volts.max():.3f} V"
            )

            with lock:
                rolling_volts = np.concatenate((rolling_volts, volts))

                if len(rolling_volts) > ROLLING_BUFFER_SAMPLES:
                    rolling_volts = rolling_volts[-ROLLING_BUFFER_SAMPLES:]

                latest_status = status

    finally:
        ser.close()


class ScopeWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Pico ADC Scope")

        self.paused = False
        self.trigger_enabled = True
        self.trigger_level = 0.609
        self.trigger_mode = "Rising"
        self.trigger_holdoff_samples = 30

        self.trace_length = 8192
        self.follow_latest = True

        self.smoothing_samples = 3
        self.peak_smoothing_samples = 100

        self.notch_freq_hz = 120000
        self.notch_r = 0.92

        self.pre_trigger_samples = 5
        self.post_trigger_samples = 50
        self.baseline_samples = 5

        self.max_pulses_to_plot = 30
        self.pulse_display_mode = "Peak Trend"

        self.last_plot_x = np.array([], dtype=np.float32)
        self.last_plot_y = np.array([], dtype=np.float32)

        layout = QtWidgets.QVBoxLayout(self)

        self.plot_widget = pg.GraphicsLayoutWidget()

        self.plot = self.plot_widget.addPlot(
            title="Waiting for data..."
        )

        self.plot.setLabel("bottom", "Sample")
        self.plot.setLabel("left", "Voltage", units="V")

        self.plot.setYRange(0, 1.0)
        self.plot.setXRange(0, 20)

        self.plot.showGrid(x=True, y=True)

        self.curve = self.plot.plot()
        self.pulse_curves = []

        self.trigger_line = pg.InfiniteLine(
            pos=self.trigger_level,
            angle=0,
            movable=True,
            pen=pg.mkPen(width=2)
        )

        self.trigger_line.sigPositionChanged.connect(
            self.update_trigger_from_line
        )

        self.plot.addItem(self.trigger_line)

        layout.addWidget(self.plot_widget)

        controls = QtWidgets.QGridLayout()

        self.pause_button = QtWidgets.QPushButton("Pause")
        self.pause_button.clicked.connect(self.toggle_pause)
        controls.addWidget(self.pause_button, 0, 0)

        self.export_button = QtWidgets.QPushButton("Export CSV")
        self.export_button.clicked.connect(self.export_csv)
        controls.addWidget(self.export_button, 0, 1)

        self.trigger_checkbox = QtWidgets.QCheckBox("Trigger")
        self.trigger_checkbox.setChecked(True)
        self.trigger_checkbox.stateChanged.connect(
            self.update_trigger_enabled
        )
        controls.addWidget(self.trigger_checkbox, 0, 2)

        self.trigger_level_label = QtWidgets.QLabel(
            f"Trigger: {self.trigger_level:.3f} V"
        )
        controls.addWidget(self.trigger_level_label, 0, 3)

        controls.addWidget(QtWidgets.QLabel("Trigger mode"), 0, 4)

        self.trigger_mode_combo = QtWidgets.QComboBox()
        self.trigger_mode_combo.addItems([
            "Rising",
            "Falling",
            "Either"
        ])
        self.trigger_mode_combo.currentTextChanged.connect(
            self.update_trigger_mode
        )
        controls.addWidget(self.trigger_mode_combo, 0, 5)

        controls.addWidget(QtWidgets.QLabel("Y min"), 1, 0)

        self.y_min_spin = QtWidgets.QDoubleSpinBox()
        self.y_min_spin.setRange(-1.0, 3.3)
        self.y_min_spin.setSingleStep(0.05)
        self.y_min_spin.setValue(0)
        self.y_min_spin.valueChanged.connect(self.update_y_range)
        controls.addWidget(self.y_min_spin, 1, 1)

        controls.addWidget(QtWidgets.QLabel("Y max"), 1, 2)

        self.y_max_spin = QtWidgets.QDoubleSpinBox()
        self.y_max_spin.setRange(0.0, 5.0)
        self.y_max_spin.setSingleStep(0.05)
        self.y_max_spin.setValue(0.2)
        self.y_max_spin.valueChanged.connect(self.update_y_range)
        controls.addWidget(self.y_max_spin, 1, 3)

        controls.addWidget(QtWidgets.QLabel("Holdoff"), 1, 4)

        self.trigger_holdoff_spin = QtWidgets.QSpinBox()
        self.trigger_holdoff_spin.setRange(0, ROLLING_BUFFER_SAMPLES)
        self.trigger_holdoff_spin.setValue(self.trigger_holdoff_samples)
        self.trigger_holdoff_spin.valueChanged.connect(
            self.update_trigger_holdoff
        )
        controls.addWidget(self.trigger_holdoff_spin, 1, 5)

        self.trace_label = QtWidgets.QLabel(
            f"Trace length: {self.trace_length}"
        )
        controls.addWidget(self.trace_label, 2, 0)

        self.trace_slider = QtWidgets.QSlider(
            QtCore.Qt.Orientation.Horizontal
        )
        self.trace_slider.setRange(16, ROLLING_BUFFER_SAMPLES)
        self.trace_slider.setSingleStep(16)
        self.trace_slider.setPageStep(512)
        self.trace_slider.setValue(self.trace_length)
        self.trace_slider.valueChanged.connect(self.update_trace_length)
        controls.addWidget(self.trace_slider, 2, 1)

        self.refresh_label = QtWidgets.QLabel("Refresh: 33 ms")
        controls.addWidget(self.refresh_label, 2, 2)

        self.refresh_slider = QtWidgets.QSlider(
            QtCore.Qt.Orientation.Horizontal
        )
        self.refresh_slider.setRange(5, 1000)
        self.refresh_slider.setSingleStep(5)
        self.refresh_slider.setPageStep(50)
        self.refresh_slider.setValue(33)
        self.refresh_slider.valueChanged.connect(self.update_refresh_rate)
        controls.addWidget(self.refresh_slider, 2, 3)

        self.follow_checkbox = QtWidgets.QCheckBox("Follow latest")
        self.follow_checkbox.setChecked(True)
        self.follow_checkbox.stateChanged.connect(self.update_follow_latest)
        controls.addWidget(self.follow_checkbox, 2, 4)

        self.smoothing_checkbox = QtWidgets.QCheckBox("Smooth waveform")
        self.smoothing_checkbox.setChecked(True)
        controls.addWidget(self.smoothing_checkbox, 3, 0)

        self.smoothing_label = QtWidgets.QLabel(
            f"Smooth: {self.smoothing_samples}"
        )
        controls.addWidget(self.smoothing_label, 3, 1)

        self.smoothing_slider = QtWidgets.QSlider(
            QtCore.Qt.Orientation.Horizontal
        )
        self.smoothing_slider.setRange(1, 101)
        self.smoothing_slider.setSingleStep(2)
        self.smoothing_slider.setPageStep(10)
        self.smoothing_slider.setValue(self.smoothing_samples)
        self.smoothing_slider.valueChanged.connect(
            self.update_smoothing_samples
        )
        controls.addWidget(self.smoothing_slider, 3, 2, 1, 4)

        self.notch_checkbox = QtWidgets.QCheckBox("Notch")
        self.notch_checkbox.setChecked(True)
        controls.addWidget(self.notch_checkbox, 4, 0)

        self.notch_label = QtWidgets.QLabel(
            f"Notch: {self.notch_freq_hz} Hz"
        )
        controls.addWidget(self.notch_label, 4, 1)

        self.notch_freq_slider = QtWidgets.QSlider(
            QtCore.Qt.Orientation.Horizontal
        )
        self.notch_freq_slider.setRange(
            1000,
            (ADC_SAMPLE_RATE_HZ // 2) - 1000
        )
        self.notch_freq_slider.setSingleStep(1000)
        self.notch_freq_slider.setPageStep(10000)
        self.notch_freq_slider.setValue(self.notch_freq_hz)
        self.notch_freq_slider.valueChanged.connect(
            self.update_notch_freq
        )
        controls.addWidget(self.notch_freq_slider, 4, 2, 1, 2)

        self.notch_q_label = QtWidgets.QLabel(
            f"Notch Q: {self.notch_r:.3f}"
        )
        controls.addWidget(self.notch_q_label, 4, 4)

        self.notch_q_slider = QtWidgets.QSlider(
            QtCore.Qt.Orientation.Horizontal
        )
        self.notch_q_slider.setRange(800, 995)
        self.notch_q_slider.setSingleStep(5)
        self.notch_q_slider.setPageStep(25)
        self.notch_q_slider.setValue(int(self.notch_r * 1000))
        self.notch_q_slider.valueChanged.connect(
            self.update_notch_q
        )
        controls.addWidget(self.notch_q_slider, 4, 5)

        self.pulse_only_checkbox = QtWidgets.QCheckBox("Pulse-only")
        self.pulse_only_checkbox.setChecked(True)
        self.pulse_only_checkbox.stateChanged.connect(
            self.update_pulse_only_mode
        )
        controls.addWidget(self.pulse_only_checkbox, 5, 0)

        self.baseline_checkbox = QtWidgets.QCheckBox(
            "Subtract baseline"
        )
        self.baseline_checkbox.setChecked(True)
        controls.addWidget(self.baseline_checkbox, 5, 1)

        controls.addWidget(QtWidgets.QLabel("Pulse display"), 5, 2)

        self.pulse_display_combo = QtWidgets.QComboBox()
        self.pulse_display_combo.addItems([
            "Overlay",
            "Sequential",
            "Peak trend"
        ])
        self.pulse_display_combo.currentTextChanged.connect(
            self.update_pulse_display_mode
        )
        controls.addWidget(self.pulse_display_combo, 5, 3)

        controls.addWidget(QtWidgets.QLabel("Pre samples"), 6, 0)

        self.pre_trigger_spin = QtWidgets.QSpinBox()
        self.pre_trigger_spin.setRange(0, ROLLING_BUFFER_SAMPLES - 2)
        self.pre_trigger_spin.setSingleStep(1)
        self.pre_trigger_spin.setValue(self.pre_trigger_samples)
        self.pre_trigger_spin.valueChanged.connect(
            self.update_pre_trigger_samples
        )
        controls.addWidget(self.pre_trigger_spin, 6, 1)

        controls.addWidget(QtWidgets.QLabel("Post samples"), 6, 2)

        self.post_trigger_spin = QtWidgets.QSpinBox()
        self.post_trigger_spin.setRange(1, ROLLING_BUFFER_SAMPLES - 1)
        self.post_trigger_spin.setSingleStep(1)
        self.post_trigger_spin.setValue(self.post_trigger_samples)
        self.post_trigger_spin.valueChanged.connect(
            self.update_post_trigger_samples
        )
        controls.addWidget(self.post_trigger_spin, 6, 3)

        controls.addWidget(QtWidgets.QLabel("Baseline samples"), 7, 0)

        self.baseline_spin = QtWidgets.QSpinBox()
        self.baseline_spin.setRange(1, ROLLING_BUFFER_SAMPLES)
        self.baseline_spin.setSingleStep(1)
        self.baseline_spin.setValue(self.baseline_samples)
        self.baseline_spin.valueChanged.connect(
            self.update_baseline_samples
        )
        controls.addWidget(self.baseline_spin, 7, 1)

        controls.addWidget(QtWidgets.QLabel("Max pulses"), 7, 2)

        self.max_pulses_spin = QtWidgets.QSpinBox()
        self.max_pulses_spin.setRange(1, 200)
        self.max_pulses_spin.setSingleStep(1)
        self.max_pulses_spin.setValue(self.max_pulses_to_plot)
        self.max_pulses_spin.valueChanged.connect(self.update_max_pulses)
        controls.addWidget(self.max_pulses_spin, 7, 3)

        self.peak_smoothing_label = QtWidgets.QLabel(
            f"Peak smooth: {self.peak_smoothing_samples}"
        )
        controls.addWidget(self.peak_smoothing_label, 8, 0)

        self.peak_smoothing_slider = QtWidgets.QSlider(
            QtCore.Qt.Orientation.Horizontal
        )
        self.peak_smoothing_slider.setRange(1, 100)
        self.peak_smoothing_slider.setSingleStep(1)
        self.peak_smoothing_slider.setPageStep(10)
        self.peak_smoothing_slider.setValue(
            self.peak_smoothing_samples
        )
        self.peak_smoothing_slider.valueChanged.connect(
            self.update_peak_smoothing_samples
        )
        controls.addWidget(self.peak_smoothing_slider, 8, 1, 1, 3)

        self.peak_meter_label = QtWidgets.QLabel(
            "Average peak: n/a"
        )
        controls.addWidget(self.peak_meter_label, 9, 0)

        self.peak_meter = QtWidgets.QProgressBar()
        self.peak_meter.setRange(0, int(200))
        self.peak_meter.setValue(0)
        self.peak_meter.setFormat("%v mV")
        controls.addWidget(self.peak_meter, 9, 1, 1, 3)

        layout.addLayout(controls)

        self.update_y_range()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(33)

    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_button.setText(
            "Run" if self.paused else "Pause"
        )

    def export_csv(self):
        if not self.paused:
            QtWidgets.QMessageBox.warning(
                self,
                "Export CSV",
                "Pause the trace before exporting."
            )
            return

        if len(self.last_plot_x) == 0 or len(self.last_plot_y) == 0:
            QtWidgets.QMessageBox.warning(
                self,
                "Export CSV",
                "No trace data available to export."
            )
            return

        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Export paused trace",
            "paused_trace.csv",
            "CSV files (*.csv)"
        )

        if not path:
            return

        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "sample",
                "time_s",
                "voltage_v"
            ])

            for sample, voltage in zip(self.last_plot_x, self.last_plot_y):
                writer.writerow([
                    int(sample),
                    float(sample) / float(ADC_SAMPLE_RATE_HZ),
                    float(voltage)
                ])

    def update_trigger_enabled(self, state):
        self.trigger_enabled = (
            state == QtCore.Qt.CheckState.Checked.value
        )
        self.trigger_line.setVisible(self.trigger_enabled)

    def update_trigger_from_line(self):
        value = float(self.trigger_line.value())
        value = max(0.0, min(VREF, value))

        self.trigger_level = value

        self.trigger_level_label.setText(
            f"Trigger: {self.trigger_level:.3f} V"
        )

    def update_trigger_mode(self, value):
        self.trigger_mode = value

    def update_trigger_holdoff(self, value):
        self.trigger_holdoff_samples = value

    def update_y_range(self):
        y_min = self.y_min_spin.value()
        y_max = self.y_max_spin.value()

        if y_max <= y_min:
            return

        self.plot.setYRange(y_min, y_max, padding=0)

    def update_trace_length(self, value):
        value = max(16, int(value))
        value = value - (value % 16)

        self.trace_length = value

        self.trace_label.setText(
            f"Trace length: {self.trace_length}"
        )

        if not self.pulse_only_checkbox.isChecked() and self.follow_latest:
            self.plot.setXRange(
                -50,
                150,
                padding=0
            )

    def update_follow_latest(self, state):
        self.follow_latest = (
            state == QtCore.Qt.CheckState.Checked.value
        )

        if self.follow_latest and not self.pulse_only_checkbox.isChecked():
            self.plot.setXRange(
                -self.trace_length,
                0,
                padding=0
            )

    def update_refresh_rate(self, value):
        value = max(5, int(value))

        self.refresh_label.setText(
            f"Refresh: {value} ms"
        )

        self.timer.start(value)

    def update_smoothing_samples(self, value):
        if value % 2 == 0:
            value += 1

            self.smoothing_slider.blockSignals(True)
            self.smoothing_slider.setValue(value)
            self.smoothing_slider.blockSignals(False)

        self.smoothing_samples = value

        self.smoothing_label.setText(
            f"Smooth: {self.smoothing_samples}"
        )

    def update_notch_freq(self, value):
        self.notch_freq_hz = int(value)

        self.notch_label.setText(
            f"Notch: {self.notch_freq_hz} Hz"
        )

    def update_notch_q(self, value):
        self.notch_r = float(value) / 1000.0

        self.notch_q_label.setText(
            f"Notch Q: {self.notch_r:.3f}"
        )

    def update_peak_smoothing_samples(self, value):
        self.peak_smoothing_samples = int(value)

        self.peak_smoothing_label.setText(
            f"Peak smooth: {self.peak_smoothing_samples}"
        )

    def update_pulse_display_mode(self, value):
        self.pulse_display_mode = value
        self.update_pulse_x_range()

    def update_pre_trigger_samples(self, value):
        self.pre_trigger_samples = value
        self.update_pulse_x_range()

    def update_post_trigger_samples(self, value):
        self.post_trigger_samples = value
        self.update_pulse_x_range()

    def update_baseline_samples(self, value):
        self.baseline_samples = value

    def update_max_pulses(self, value):
        self.max_pulses_to_plot = value
        self.update_pulse_x_range()

    def update_pulse_only_mode(self):
        self.clear_pulse_curves()

        if self.pulse_only_checkbox.isChecked():
            self.curve.setData([])
            self.update_pulse_x_range()

        else:
            if self.follow_latest:
                self.plot.setXRange(
                    -self.trace_length,
                    0,
                    padding=0
                )

    def update_pulse_x_range(self):
        if not self.pulse_only_checkbox.isChecked():
            return

        if self.pulse_display_mode == "Overlay":
            self.plot.setXRange(
                -self.pre_trigger_samples,
                self.post_trigger_samples,
                padding=0
            )

        elif self.pulse_display_mode == "Sequential":
            pulse_len = (
                self.pre_trigger_samples
                + self.post_trigger_samples
            )

            self.plot.setXRange(
                0,
                max(1, pulse_len * self.max_pulses_to_plot),
                padding=0
            )

        else:
            self.plot.setXRange(
                0,
                max(1, self.max_pulses_to_plot - 1),
                padding=0
            )

    def moving_average(self, values, window):
        if len(values) == 0:
            return values

        if window <= 1:
            return values

        window = min(window, len(values))

        kernel = np.ones(window, dtype=np.float32) / window

        pad_left = window // 2
        pad_right = window - 1 - pad_left

        padded = np.pad(
            values,
            (pad_left, pad_right),
            mode="edge"
        )

        return np.convolve(padded, kernel, mode="valid")

    def notch_filter(self, values):
        if not self.notch_checkbox.isChecked():
            return values

        if len(values) < 3:
            return values

        f0 = float(self.notch_freq_hz)
        fs = float(ADC_SAMPLE_RATE_HZ)

        if f0 <= 0.0 or f0 >= fs / 2.0:
            return values

        r = self.notch_r
        w0 = 2.0 * np.pi * f0 / fs
        c = np.cos(w0)

        y = np.empty_like(values, dtype=np.float32)

        x1 = 0.0
        x2 = 0.0
        y1 = 0.0
        y2 = 0.0

        for i, x0 in enumerate(values.astype(np.float32)):
            y0 = (
                x0
                - 2.0 * c * x1
                + x2
                + 2.0 * r * c * y1
                - (r * r) * y2
            )

            y[i] = y0

            x2 = x1
            x1 = x0
            y2 = y1
            y1 = y0

        return y

    def smooth_waveform(self, volts):
        volts = self.notch_filter(volts)

        if not self.smoothing_checkbox.isChecked():
            return volts

        return self.moving_average(
            volts,
            self.smoothing_samples
        ).astype(np.float32)

    def find_trigger_crossings(self, volts):
        trigger = self.trigger_level

        if self.trigger_mode == "Rising":
            crossings = np.where(
                (volts[:-1] < trigger)
                & (volts[1:] >= trigger)
            )[0] + 1

        elif self.trigger_mode == "Falling":
            crossings = np.where(
                (volts[:-1] > trigger)
                & (volts[1:] <= trigger)
            )[0] + 1

        else:
            rising = np.where(
                (volts[:-1] < trigger)
                & (volts[1:] >= trigger)
            )[0] + 1

            falling = np.where(
                (volts[:-1] > trigger)
                & (volts[1:] <= trigger)
            )[0] + 1

            crossings = np.sort(
                np.concatenate((rising, falling))
            )

        if self.trigger_holdoff_samples <= 0:
            return crossings

        accepted = []
        last = -self.trigger_holdoff_samples

        for idx in crossings:
            if idx - last >= self.trigger_holdoff_samples:
                accepted.append(idx)
                last = idx

        return np.array(accepted, dtype=np.int64)

    def align_to_trigger(self, volts):
        if not self.trigger_enabled:
            return volts

        trigger_volts = self.smooth_waveform(volts)
        crossings = self.find_trigger_crossings(trigger_volts)

        if len(crossings) == 0:
            return volts

        idx = crossings[0]

        return np.roll(volts, -idx)

    def extract_pulses(self, volts):
        trigger_volts = self.smooth_waveform(volts)
        crossings = self.find_trigger_crossings(trigger_volts)

        pulses = []

        pre = self.pre_trigger_samples
        post = self.post_trigger_samples

        for idx in crossings:
            start = idx - pre
            end = idx + post

            if start < 0:
                continue

            if end > len(volts):
                continue

            pulse = volts[start:end].copy()

            if self.baseline_checkbox.isChecked():
                baseline_end = pre
                baseline_start = max(
                    0,
                    baseline_end - self.baseline_samples
                )

                if baseline_end > baseline_start:
                    baseline = np.mean(
                        pulse[baseline_start:baseline_end]
                    )

                    pulse = pulse - baseline

            pulse = self.smooth_waveform(pulse)

            pulses.append(pulse)

        if len(pulses) > self.max_pulses_to_plot:
            pulses = pulses[-self.max_pulses_to_plot:]

        return pulses

    def get_pulse_peaks(self, pulses):
        if len(pulses) == 0:
            return np.array([], dtype=np.float32)

        peaks = [np.max(pulse) for pulse in pulses]

        return np.array(peaks, dtype=np.float32)

    def get_smoothed_peaks(self, peaks):
        return self.moving_average(
            peaks,
            self.peak_smoothing_samples
        ).astype(np.float32)

    def get_pulse_peak_stats(self, pulses):
        peaks = self.get_pulse_peaks(pulses)

        if len(peaks) == 0:
            return None

        smoothed_peaks = self.get_smoothed_peaks(peaks)

        return {
            "avg": float(np.mean(smoothed_peaks)),
            "min": float(np.min(smoothed_peaks)),
            "max": float(np.max(smoothed_peaks)),
            "raw_avg": float(np.mean(peaks)),
        }

    def update_peak_meter(self, peak_stats):
        if peak_stats is None:
            self.peak_meter.setValue(0)
            self.peak_meter_label.setText(
                "Average peak: n/a"
            )
            return

        avg_peak = peak_stats["avg"]
        avg_mv = int(round(avg_peak * 1000))

        self.peak_meter.setValue(
            max(0, min(int(VREF * 1000), avg_mv))
        )

        self.peak_meter_label.setText(
            f"Average peak: {avg_peak:.3f} V"
        )

    def clear_pulse_curves(self):
        for curve in self.pulse_curves:
            self.plot.removeItem(curve)

        self.pulse_curves = []

    def plot_pulses_overlay(self, pulses):
        x = np.arange(
            -self.pre_trigger_samples,
            self.post_trigger_samples,
            dtype=np.int32
        )

        for pulse in pulses:
            curve = self.plot.plot(
                x,
                pulse,
                pen=pg.mkPen(width=1)
            )

            self.pulse_curves.append(curve)

    def plot_pulses_sequential(self, pulses):
        pulse_len = (
            self.pre_trigger_samples
            + self.post_trigger_samples
        )

        xs = []
        ys = []

        for i, pulse in enumerate(pulses):
            start_x = i * pulse_len

            pulse_x = (
                start_x
                + np.arange(pulse_len, dtype=np.float32)
            )

            pulse_y = pulse.astype(np.float32)

            xs.append(pulse_x)
            ys.append(pulse_y)

        if len(xs) == 0:
            return

        x = np.concatenate(xs)
        y = np.concatenate(ys)

        curve = self.plot.plot(
            x,
            y,
            pen=pg.mkPen(width=1)
        )

        self.pulse_curves.append(curve)

    def plot_peak_trend(self, pulses):
        raw_peaks = self.get_pulse_peaks(pulses)

        if len(raw_peaks) == 0:
            return

        smoothed_peaks = self.get_smoothed_peaks(raw_peaks)

        x = np.arange(len(raw_peaks), dtype=np.float32)

        raw_curve = self.plot.plot(
            x,
            raw_peaks,
            pen=None,
            symbol="o",
            symbolSize=5
        )

        self.pulse_curves.append(raw_curve)

        smooth_curve = self.plot.plot(
            x,
            smoothed_peaks,
            pen=pg.mkPen(width=2)
        )

        self.pulse_curves.append(smooth_curve)

    def plot_pulses(self, pulses):
        self.curve.setData([])
        self.clear_pulse_curves()

        if self.pulse_display_mode == "Overlay":
            self.plot_pulses_overlay(pulses)

        elif self.pulse_display_mode == "Sequential":
            self.plot_pulses_sequential(pulses)

        else:
            self.plot_peak_trend(pulses)

    def update_plot(self):
        if self.paused:
            return

        with lock:
            if len(rolling_volts) == 0:
                return

            volts = rolling_volts.copy()
            status = latest_status

        if self.pulse_only_checkbox.isChecked():
            pulses = self.extract_pulses(volts)

            self.plot_pulses(pulses)

            peak_stats = self.get_pulse_peak_stats(pulses)

            self.update_peak_meter(peak_stats)

            if peak_stats is None:
                peak_text = "avg peak=n/a"

            else:
                peak_text = (
                    f"avg peak={peak_stats['avg']:.3f} V | "
                    f"raw avg={peak_stats['raw_avg']:.3f} V | "
                    f"min={peak_stats['min']:.3f} V | "
                    f"max={peak_stats['max']:.3f} V"
                )

            self.plot.setTitle(
                f"{status} | buffer={len(volts)} | "
                f"pulses={len(pulses)} | "
                f"{self.pulse_display_mode.lower()} | "
                f"{peak_text}"
            )

            return

        self.update_peak_meter(None)
        self.clear_pulse_curves()

        if len(volts) > self.trace_length:
            volts = volts[-self.trace_length:]

        volts = self.align_to_trigger(volts)
        volts = self.smooth_waveform(volts)

        x = np.arange(
            -len(volts) + 1,
            1,
            dtype=np.int32
        )

        self.curve.setData(x, volts)

        self.last_plot_x = x.copy()
        self.last_plot_y = volts.copy()

        if self.follow_latest:
            self.plot.setXRange(
                -self.trace_length,
                0,
                padding=0
            )

        self.plot.setTitle(
            f"{status} | buffer={len(rolling_volts)}"
        )


app = QtWidgets.QApplication(sys.argv)

reader = threading.Thread(
    target=serial_thread,
    daemon=True
)

reader.start()

window = ScopeWindow()

window.resize(1200, 780)
window.show()


def cleanup():
    global running

    running = False

    reader.join(timeout=1)


app.aboutToQuit.connect(cleanup)

sys.exit(app.exec())