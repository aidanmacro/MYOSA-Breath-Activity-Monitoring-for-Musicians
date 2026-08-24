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

UDP_IP = "0.0.0.0"  
UDP_PORT = 12345

EXPECTED_SAMPLES = 700
MAGIC_BYTES = b"OCIP!CDA"
HEADER_REST_FMT = "<I I H H f f"
HEADER_REST_SIZE = struct.calcsize(HEADER_REST_FMT)

VREF = 3.3
ADC_MAX = 4095.0
ADC_SAMPLE_RATE_HZ = 100000 # 200kHz interleaved = 100kHz per channel

ROLLING_ADC_SAMPLES = 8192
ROLLING_BARO_POINTS = 600  

lock = threading.Lock()
running = True

rolling_volts = np.array([], dtype=np.float32)
rolling_pwm = np.array([], dtype=np.uint8)
latest_status = "Waiting..."

baro_t = deque(maxlen=ROLLING_BARO_POINTS)      
baro_temp = deque(maxlen=ROLLING_BARO_POINTS)   
baro_pressure = deque(maxlen=ROLLING_BARO_POINTS)  
baro_start_time = None

def checksum_u16(adc_u16):
    return int(np.sum(adc_u16, dtype=np.uint32) & 0xFFFF)

def udp_thread():
    global rolling_volts, rolling_pwm, latest_status, baro_start_time

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Drastically increase OS receive buffer to prevent bursts dropping
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1048576)
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

            if len(data) != expected_size or data[:len(MAGIC_BYTES)] != MAGIC_BYTES:
                continue

            packet_start = len(MAGIC_BYTES)
            rest = data[packet_start : packet_start + HEADER_REST_SIZE]
            raw = data[packet_start + HEADER_REST_SIZE :]

            _sequence, _dropped, samples, checksum, temperature, pressure = struct.unpack(HEADER_REST_FMT, rest)

            if samples != EXPECTED_SAMPLES:
                continue

            adc_u16 = np.frombuffer(raw, dtype="<u2").copy()
            if checksum_u16(adc_u16) != checksum:
                continue

            # DMA packs the channel ID into the top 4 bits of the 16-bit word
            channel_id = (adc_u16 >> 12) & 0x0F
            adc_val = adc_u16 & 0x0FFF

            # Demux the interleaved channels
            mask_pd = (channel_id == 0)   # GPIO 36
            mask_pwm = (channel_id == 3)  # GPIO 39

            pd_volts = adc_val[mask_pd].astype(np.float32) * (VREF / ADC_MAX)
            pwm_analog = adc_val[mask_pwm].astype(np.float32) * (VREF / ADC_MAX)
            
            # Reconstruct digital state (threshold > 1.5V)
            pwm_state = (pwm_analog > 1.5).astype(np.uint8)

            # Ensure arrays match length before plotting
            min_len = min(len(pd_volts), len(pwm_state))
            pd_volts = pd_volts[:min_len]
            pwm_state = pwm_state[:min_len]

            if min_len == 0:
                continue

            status = f"min={pd_volts.min():.3f} V | max={pd_volts.max():.3f} V | Dropped={_dropped}"
            now = time.time()

            with lock:
                rolling_volts = np.concatenate((rolling_volts, pd_volts))
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

    finally:
        sock.close()

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

        root = QtWidgets.QVBoxLayout(self)

        self.plot_widget = pg.GraphicsLayoutWidget()

        self.adc_plot = self.plot_widget.addPlot(row=0, col=0, title="Waiting for data...")
        self.adc_plot.setLabel("bottom", "Sample")
        self.adc_plot.setLabel("left", "Voltage", units="V")
        self.adc_plot.setYRange(0, VREF)
        self.adc_plot.showGrid(x=True, y=True)
        self.adc_curve = self.adc_plot.plot(pen=pg.mkPen(width=1))

        self.pwm_plot = self.plot_widget.addPlot(row=1, col=0, title="PWM Pin State")
        self.pwm_plot.setLabel("bottom", "Sample")
        self.pwm_plot.setLabel("left", "State")
        self.pwm_plot.setYRange(-0.2, 1.2, padding=0)
        self.pwm_plot.getAxis("left").setTicks([[(0, "0"), (1, "1")]])
        self.pwm_plot.showGrid(x=True, y=True)
        self.pwm_plot.setXLink(self.adc_plot.getViewBox())
        self.pwm_curve = self.pwm_plot.plot(pen=pg.mkPen(width=2, color="y"), stepMode="right")

        self.temp_plot = self.plot_widget.addPlot(row=2, col=0, title="Temperature")
        self.temp_plot.setLabel("bottom", "Time", units="s")
        self.temp_plot.setLabel("left", "Temp", units="\u00b0C")
        self.temp_plot.showGrid(x=True, y=True)
        self.temp_curve = self.temp_plot.plot(pen=pg.mkPen(width=2, color="r"), symbol="o", symbolSize=4)

        self.pressure_plot = self.plot_widget.addPlot(row=3, col=0, title="Pressure")
        self.pressure_plot.setLabel("bottom", "Time", units="s")
        self.pressure_plot.setLabel("left", "Pressure", units="Pa")
        self.pressure_plot.showGrid(x=True, y=True)
        self.pressure_plot.setXLink(self.temp_plot.getViewBox())
        self.pressure_curve = self.pressure_plot.plot(pen=pg.mkPen(width=2, color="c"), symbol="o", symbolSize=4)

        self.plot_widget.ci.layout.setRowStretchFactor(0, 3)
        self.plot_widget.ci.layout.setRowStretchFactor(1, 1)
        self.plot_widget.ci.layout.setRowStretchFactor(2, 1)
        self.plot_widget.ci.layout.setRowStretchFactor(3, 1)

        root.addWidget(self.plot_widget, stretch=1)

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

        root.addLayout(controls)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(33)

    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_button.setText("Run" if self.paused else "Pause")

    def update_follow_latest(self, state):
        self.follow_latest = state == QtCore.Qt.CheckState.Checked.value

    def update_trace_length(self, value):
        value = max(64, int(value))
        self.trace_length = value
        self.trace_label.setText(f"{value}")

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
                    writer.writerow([int(sample), float(sample) / float(ADC_SAMPLE_RATE_HZ), float(voltage), pwm_val])

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

    def update_plot(self):
        if self.paused:
            return

        with lock:
            volts = rolling_volts.copy()
            pwm = rolling_pwm.copy()
            status = latest_status
            buffer_len = len(rolling_volts) 
            t = list(baro_t)
            temp = list(baro_temp)
            pressure = list(baro_pressure)

        if len(volts):
            if len(volts) > self.trace_length:
                volts = volts[-self.trace_length:]
                pwm = pwm[-self.trace_length:]

            x = np.arange(0, len(volts), dtype=np.int32)
            self.adc_curve.setData(x, volts)
            self.pwm_curve.setData(x, pwm.astype(np.float32))

            self.last_plot_x = x.copy()
            self.last_plot_y = volts.copy()
            self.last_plot_pwm = pwm.copy()

            if self.follow_latest:
                self.adc_plot.setXRange(0, self.trace_length, padding=0)

            self.adc_plot.setTitle(f"{status} | buffer={buffer_len}")

        if t:
            self.temp_curve.setData(t, temp)
            self.pressure_curve.setData(t, pressure)
            self.status_label.setText(f"Temp: {temp[-1]:.2f} \u00b0C | Pressure: {pressure[-1]:.1f} Pa")

def main():
    global running
    app = QtWidgets.QApplication(sys.argv)
    reader = threading.Thread(target=udp_thread, daemon=True)
    reader.start()

    window = ScopeWindow()
    window.resize(1200, 860)
    window.show()

    def cleanup():
        global running
        running = False
        reader.join(timeout=1)

    app.aboutToQuit.connect(cleanup)
    sys.exit(app.exec())

if __name__ == "__main__":
    main()