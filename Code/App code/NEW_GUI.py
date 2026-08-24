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
UDP_IP = "0.0.0.0"  
UDP_PORT = 12345

# --- Packet configuration ---
EXPECTED_SAMPLES = 512
MAGIC_BYTES = b"OCIP!CDA"
HEADER_REST_FMT = "<I I H H f f"
HEADER_REST_SIZE = struct.calcsize(HEADER_REST_FMT)

# --- ADC scaling ---
VREF = 3.3
ADC_MAX = 4095.0
ADC_SAMPLE_RATE_HZ = 200000

# --- Rolling buffer sizes ---
ROLLING_ADC_SAMPLES = 16384
ROLLING_BARO_POINTS = 600
ROLLING_LED_POINTS = 600      

# --- Averaging & Smoothing Parameters ---
LED_AVG_WINDOW_S = 0.05
LED_AVG_WINDOW_SAMPLES = int(LED_AVG_WINDOW_S * ADC_SAMPLE_RATE_HZ) 
DELTA_V_MEDIAN_LEN = 5
DEFAULT_SMOOTHING_TAU_S = 0.30
MIN_SMOOTHING_TAU_S = 0.02
MAX_SMOOTHING_TAU_S = 2.00
DISPLAY_WINDOW_S = 5.0

# --- Temperature Correction ---
ALPHA_PD_PER_DEGC = -0.001
T0_REF_DEGC = 25.0

# --- Shared state (guarded by `lock`) ---
lock = threading.Lock()
running = True

rolling_volts = np.array([], dtype=np.float32)

baro_t = deque(maxlen=ROLLING_BARO_POINTS)
baro_temp = deque(maxlen=ROLLING_BARO_POINTS)
baro_pressure = deque(maxlen=ROLLING_BARO_POINTS)
latest_temp = None
latest_pressure = None

session_start_time = None

def get_session_start_time(now):
    global session_start_time
    with lock:
        if session_start_time is None:
            session_start_time = now
        return session_start_time

def checksum_u16(adc_u16):
    return int(np.sum(adc_u16, dtype=np.uint32) & 0xFFFF)


class SignalConditioner:
    """Two-stage filter for a noisy scalar measurement stream."""
    def __init__(self, median_len=DELTA_V_MEDIAN_LEN, tau_s=DEFAULT_SMOOTHING_TAU_S):
        self.median_len = median_len
        self.tau_s = tau_s
        self._raw_history = deque(maxlen=median_len)
        self._filtered_value = None
        self._last_t = None

    def reset(self):
        self._raw_history.clear()
        self._filtered_value = None
        self._last_t = None

    def update(self, value, t):
        if value is None:
            return self._filtered_value
        self._raw_history.append(value)
        median_value = float(np.median(self._raw_history))
        if self._filtered_value is None or self._last_t is None:
            self._filtered_value = median_value
        else:
            dt = max(t - self._last_t, 1e-4)
            alpha = 1.0 - np.exp(-dt / self.tau_s)
            self._filtered_value += alpha * (median_value - self._filtered_value)
        self._last_t = t
        return self._filtered_value


def udp_thread():
    global rolling_volts, latest_temp, latest_pressure
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

            if len(data) != expected_size or data[:len(MAGIC_BYTES)] != MAGIC_BYTES:
                continue

            packet_start = len(MAGIC_BYTES)
            rest = data[packet_start : packet_start + HEADER_REST_SIZE]
            raw = data[packet_start + HEADER_REST_SIZE :]

            (_sequence, _dropped, samples, checksum, temperature, pressure) = struct.unpack(HEADER_REST_FMT, rest)

            if samples != EXPECTED_SAMPLES:
                continue

            adc_u16 = np.frombuffer(raw, dtype="<u2").copy()
            if checksum_u16(adc_u16) != checksum:
                continue

            adc_u16 = adc_u16 & 0x0FFF
            volts = adc_u16.astype(np.float32) * (VREF / ADC_MAX)

            now = time.time()
            t0 = get_session_start_time(now)

            with lock:
                rolling_volts = np.concatenate((rolling_volts, volts))
                if len(rolling_volts) > ROLLING_ADC_SAMPLES:
                    rolling_volts = rolling_volts[-ROLLING_ADC_SAMPLES:]

                baro_t.append(now - t0)
                baro_temp.append(float(temperature))
                baro_pressure.append(float(pressure))
                latest_temp = temperature
                latest_pressure = pressure
    finally:
        sock.close()


class DeltaVMonitorWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ΔV, Temperature & Pressure Monitor")
        
        self.delta_v_filter = SignalConditioner(tau_s=DEFAULT_SMOOTHING_TAU_S)
        
        self.dv_t = deque(maxlen=ROLLING_LED_POINTS)
        self.dv_data = deque(maxlen=ROLLING_LED_POINTS)

        self.setup_ui()
        
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(33)

    def setup_ui(self):
        root = QtWidgets.QVBoxLayout(self)

        # --- Digital Display ---
        self.dv_label = QtWidgets.QLabel("--.---- V")
        digital_font = QtGui.QFont("Consolas", 56, QtGui.QFont.Weight.Bold)
        self.dv_label.setFont(digital_font)
        self.dv_label.setStyleSheet("background-color: black; color: #ff69b4; padding: 12px; border-radius: 8px;")
        self.dv_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        root.addWidget(self.dv_label)

        # --- Graphics Layout ---
        self.plot_widget = pg.GraphicsLayoutWidget()
        root.addWidget(self.plot_widget, stretch=1)

        # 1. Delta V Plot (Main)
        self.dv_plot = self.plot_widget.addPlot(row=0, col=0, title="Temp-Corrected, Smoothed \u0394V")
        self.dv_plot.setLabel("left", "Voltage", units="V")
        self.dv_plot.showGrid(x=True, y=True)
        self.dv_plot.setXRange(0, DISPLAY_WINDOW_S, padding=0)
        self.dv_curve = self.dv_plot.plot(pen=pg.mkPen(width=3, color="#ff69b4"))

        # 2. Temperature Plot
        self.temp_plot = self.plot_widget.addPlot(row=1, col=0, title="Temperature")
        self.temp_plot.setLabel("left", "Temp", units="\u00b0C")
        self.temp_plot.showGrid(x=True, y=True)
        self.temp_curve = self.temp_plot.plot(pen=pg.mkPen(width=2, color="r"))
        self.temp_plot.setXLink(self.dv_plot.getViewBox()) 

        # 3. Pressure Plot
        self.press_plot = self.plot_widget.addPlot(row=2, col=0, title="Pressure")
        self.press_plot.setLabel("bottom", "Time", units="s")
        self.press_plot.setLabel("left", "Pressure", units="Pa")
        self.press_plot.showGrid(x=True, y=True)
        self.press_curve = self.press_plot.plot(pen=pg.mkPen(width=2, color="c"))
        self.press_plot.setXLink(self.dv_plot.getViewBox()) 

        # Make Delta V plot 3x larger than Temp and Pressure plots
        self.plot_widget.ci.layout.setRowStretchFactor(0, 3)
        self.plot_widget.ci.layout.setRowStretchFactor(1, 1)
        self.plot_widget.ci.layout.setRowStretchFactor(2, 1)

    def compute_direct_delta_v(self, volts_full, current_temp):
        """Calculates Delta V using O(N) partitioning to eliminate UI lag, and applies temp correction."""
        if len(volts_full) == 0:
            return None

        n = min(len(volts_full), LED_AVG_WINDOW_SAMPLES)
        window_v = volts_full[-n:]
        
        # O(N) partitioning instead of O(N log N) sorting. Much faster than np.percentile!
        k = max(1, n // 20) # Use 5% of the window size
        v_high = np.mean(np.partition(window_v, -k)[-k:])
        v_low = np.mean(np.partition(window_v, k)[:k])
        
        raw_delta_v = float(v_high - v_low)

        # Apply temperature correction 
        if current_temp is not None:
            denom = 1.0 + ALPHA_PD_PER_DEGC * (current_temp - T0_REF_DEGC)
            if denom != 0:
                raw_delta_v = raw_delta_v / denom

        return raw_delta_v

    def update_plot(self):
        with lock:
            volts_full = rolling_volts.copy()
            t_baro = list(baro_t)
            temp = list(baro_temp)
            pressure = list(baro_pressure)
            current_temp = latest_temp

        now = time.time()
        t0 = get_session_start_time(now)

        # Get Temp-Corrected Raw Delta V
        raw_display_v = self.compute_direct_delta_v(volts_full, current_temp)

        # Apply Median + EMA filter
        filtered_display_v = self.delta_v_filter.update(raw_display_v, now)

        # Update Delta V UI
        if filtered_display_v is not None:
            self.dv_t.append(now - t0)
            self.dv_data.append(filtered_display_v)
            
            self.dv_curve.setData(list(self.dv_t), list(self.dv_data))
            self.dv_label.setText(f"{filtered_display_v:.5f} V")
            
            t_latest = self.dv_t[-1]
            self.dv_plot.setXRange(max(0, t_latest - DISPLAY_WINDOW_S), max(DISPLAY_WINDOW_S, t_latest), padding=0)

        # Update Temp and Pressure UI
        if t_baro:
            self.temp_curve.setData(t_baro, temp)
            self.press_curve.setData(t_baro, pressure)


def main():
    global running
    app = QtWidgets.QApplication(sys.argv)
    reader = threading.Thread(target=udp_thread, daemon=True)
    reader.start()
    
    window = DeltaVMonitorWindow()
    window.resize(1200, 900)
    window.show()

    def cleanup():
        global running
        running = False
        reader.join(timeout=1)

    app.aboutToQuit.connect(cleanup)
    sys.exit(app.exec())

if __name__ == "__main__":
    main()