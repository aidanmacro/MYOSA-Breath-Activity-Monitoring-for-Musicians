import socket
import struct
import sys
import threading
import time
from collections import deque

import numpy as np
import pyqtgraph as pg
from PyQt6 import QtCore, QtGui, QtWidgets

# --- UDP & Packet Configuration ---
# We listen on all interfaces (0.0.0.0) on port 12345 for incoming sensor data.
UDP_IP = "0.0.0.0"  
UDP_PORT = 12345

EXPECTED_SAMPLES = 512
MAGIC_BYTES = b"OCIP!CDA"
# The header structure: sequence (uint32), dropped (uint32), samples (uint16), 
# checksum (uint16), temperature (float32), pressure (float32).
HEADER_REST_FMT = "<I I H H f f"
HEADER_REST_SIZE = struct.calcsize(HEADER_REST_FMT)

# --- Hardware & ADC Parameters ---
VREF = 3.3
ADC_MAX = 4095.0
ADC_SAMPLE_RATE_HZ = 200000

# --- Buffer Sizes ---
# These dictate how much history we keep in memory for the rolling windows[cite: 1].
ROLLING_ADC_SAMPLES = 16384
ROLLING_BARO_POINTS = 600
ROLLING_LED_POINTS = 600      

# --- Signal Processing Parameters ---
# We average the LED data over 50ms to smooth out high-frequency noise[cite: 1].
LED_AVG_WINDOW_S = 0.05
LED_AVG_WINDOW_SAMPLES = int(LED_AVG_WINDOW_S * ADC_SAMPLE_RATE_HZ) 
DELTA_V_MEDIAN_LEN = 5
DEFAULT_SMOOTHING_TAU_S = 0.30
DISPLAY_WINDOW_S = 5.0

# --- Temperature Correction ---
# Used to adjust the voltage reading based on thermal drift[cite: 1].
ALPHA_PD_PER_DEGC = -0.001
T0_REF_DEGC = 25.0

# --- Shared State ---
# Since UDP reading and UI updating happen on different threads, we use a lock 
# to prevent race conditions when reading/writing to these shared variables[cite: 1].
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
    """Fetches the session start time, initializing it if it's the first call."""
    global session_start_time
    with lock:
        if session_start_time is None:
            session_start_time = now
        return session_start_time

def checksum_u16(adc_u16):
    """Calculates a simple 16-bit checksum for the ADC array[cite: 1]."""
    return int(np.sum(adc_u16, dtype=np.uint32) & 0xFFFF)


class SignalConditioner:
    """
    A two-stage filter that first applies a median filter to reject outliers, 
    followed by an Exponential Moving Average (EMA) to smooth the data.
    """
    def __init__(self, median_len=DELTA_V_MEDIAN_LEN, tau_s=DEFAULT_SMOOTHING_TAU_S):
        self.median_len = median_len
        self.tau_s = tau_s
        self._raw_history = deque(maxlen=median_len)
        self._filtered_value = None
        self._last_t = None

    def update(self, value, t):
        if value is None:
            return self._filtered_value
            
        self._raw_history.append(value)
        median_value = float(np.median(self._raw_history))
        
        # Initialize the filter on the first pass
        if self._filtered_value is None or self._last_t is None:
            self._filtered_value = median_value
        else:
            # Time-aware Exponential Moving Average (EMA)
            # This ensures smoothing remains consistent even if frame rates fluctuate.
            dt = max(t - self._last_t, 1e-4)
            alpha = 1.0 - np.exp(-dt / self.tau_s)
            self._filtered_value += alpha * (median_value - self._filtered_value)
            
        self._last_t = t
        return self._filtered_value


def udp_thread():
    """
    Background worker that continuously listens for incoming UDP sensor packets,
    verifies their integrity, and safely appends the data to the shared buffers.
    """
    global rolling_volts, latest_temp, latest_pressure
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(0.1)

    expected_size = len(MAGIC_BYTES) + HEADER_REST_SIZE + (EXPECTED_SAMPLES * 2)

    try:
        while running:
            try:
                data, _addr = sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break

            # Drop the packet if the size or magic bytes don't match
            if len(data) != expected_size or data[:len(MAGIC_BYTES)] != MAGIC_BYTES:
                continue

            # Slice the packet into header and payload
            packet_start = len(MAGIC_BYTES)
            rest = data[packet_start : packet_start + HEADER_REST_SIZE]
            raw = data[packet_start + HEADER_REST_SIZE :]

            _sequence, _dropped, samples, checksum, temperature, pressure = struct.unpack(HEADER_REST_FMT, rest)

            if samples != EXPECTED_SAMPLES:
                continue

            # Unpack raw ADC bytes and verify checksum
            adc_u16 = np.frombuffer(raw, dtype="<u2").copy()
            if checksum_u16(adc_u16) != checksum:
                continue

            # Mask out non-data bits and convert to actual voltage[cite: 1]
            adc_u16 = adc_u16 & 0x0FFF
            volts = adc_u16.astype(np.float32) * (VREF / ADC_MAX)

            now = time.time()
            t0 = get_session_start_time(now)

            # Safely update shared global state
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
    """Main UI Window for visualizing Voltage, Temperature, and Pressure."""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ΔV, Temperature & Pressure Monitor")
        
        self.delta_v_filter = SignalConditioner(tau_s=DEFAULT_SMOOTHING_TAU_S)
        self.dv_t = deque(maxlen=ROLLING_LED_POINTS)
        self.dv_data = deque(maxlen=ROLLING_LED_POINTS)

        self.setup_ui()
        
        # UI Refresh loop: Runs at ~30 FPS (33ms)[cite: 1]
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(33)

    def setup_ui(self):
        """Constructs the layout and PyQtGraph plots."""
        root = QtWidgets.QVBoxLayout(self)

        # Large Digital Readout for Delta V
        self.dv_label = QtWidgets.QLabel("--.---- V")
        digital_font = QtGui.QFont("Consolas", 56, QtGui.QFont.Weight.Bold)
        self.dv_label.setFont(digital_font)
        self.dv_label.setStyleSheet("background-color: black; color: #ff69b4; padding: 12px; border-radius: 8px;")
        self.dv_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        root.addWidget(self.dv_label)

        self.plot_widget = pg.GraphicsLayoutWidget()
        root.addWidget(self.plot_widget, stretch=1)

        # 1. Delta V Plot
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

        # Allocate more vertical space to the primary Delta V plot[cite: 1]
        self.plot_widget.ci.layout.setRowStretchFactor(0, 3)
        self.plot_widget.ci.layout.setRowStretchFactor(1, 1)
        self.plot_widget.ci.layout.setRowStretchFactor(2, 1)

    def compute_direct_delta_v(self, volts_full, current_temp):
        """
        Calculates the peak-to-peak voltage (Delta V) of the recent window.
        Uses O(N) partitioning for performance and applies thermal drift correction.
        """
        if len(volts_full) == 0:
            return None

        n = min(len(volts_full), LED_AVG_WINDOW_SAMPLES)
        window_v = volts_full[-n:]
        
        # Performance optimization:
        # Instead of sorting the entire array (O(N log N)), we use np.partition (O(N)).
        # This isolates the highest and lowest 5% to calculate the delta without lag[cite: 1].
        k = max(1, n // 20) 
        v_high = np.mean(np.partition(window_v, -k)[-k:])
        v_low = np.mean(np.partition(window_v, k)[:k])
        
        raw_delta_v = float(v_high - v_low)

        # Temperature Correction
        if current_temp is not None:
            denom = 1.0 + ALPHA_PD_PER_DEGC * (current_temp - T0_REF_DEGC)
            if denom != 0:
                raw_delta_v = raw_delta_v / denom

        return raw_delta_v

    def update_plot(self):
        """Called periodically by the QTimer to pull data and refresh the UI."""
        # Quickly copy shared data to avoid blocking the UDP thread
        with lock:
            volts_full = rolling_volts.copy()
            t_baro = list(baro_t)
            temp = list(baro_temp)
            pressure = list(baro_pressure)
            current_temp = latest_temp

        now = time.time()
        t0 = get_session_start_time(now)

        # Calculate and filter the Delta V
        raw_display_v = self.compute_direct_delta_v(volts_full, current_temp)
        filtered_display_v = self.delta_v_filter.update(raw_display_v, now)

        # Update the UI components
        if filtered_display_v is not None:
            self.dv_t.append(now - t0)
            self.dv_data.append(filtered_display_v)
            
            self.dv_curve.setData(list(self.dv_t), list(self.dv_data))
            self.dv_label.setText(f"{filtered_display_v:.5f} V")
            
            t_latest = self.dv_t[-1]
            self.dv_plot.setXRange(max(0, t_latest - DISPLAY_WINDOW_S), max(DISPLAY_WINDOW_S, t_latest), padding=0)

        if t_baro:
            self.temp_curve.setData(t_baro, temp)
            self.press_curve.setData(t_baro, pressure)


def main():
    """Application entry point: starts the background thread and UI event loop."""
    global running
    app = QtWidgets.QApplication(sys.argv)
    
    # Start the UDP listener in a daemon thread so it dies when the app closes
    reader = threading.Thread(target=udp_thread, daemon=True)
    reader.start()
    
    window = DeltaVMonitorWindow()
    window.resize(1200, 900)
    window.show()

    def cleanup():
        """Gracefully shuts down the background thread."""
        global running
        running = False
        reader.join(timeout=1)

    app.aboutToQuit.connect(cleanup)
    sys.exit(app.exec())

if __name__ == "__main__":
    main()