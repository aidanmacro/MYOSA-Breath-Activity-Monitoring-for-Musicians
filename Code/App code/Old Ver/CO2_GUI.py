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
# Must comfortably exceed LED_AVG_WINDOW_SAMPLES (10000) so the averaging
# window is never starved of data.
ROLLING_ADC_SAMPLES = 16384
ROLLING_LED_POINTS = 600      # ~10 min of LED on/off + photocurrent history
ROLLING_CO2_POINTS = 4000     # ~2+ min of CO2 history at ~30 Hz

# --- Historical averaging window for the LED on/off split ---
# Widened from the old 10 ms window: at 200 kSPS a longer window spans many
# more PWM cycles, which averages down ADC/EMI noise a lot faster than a
# short window ever can.
LED_AVG_WINDOW_S = 0.05
LED_AVG_WINDOW_SAMPLES = int(LED_AVG_WINDOW_S * ADC_SAMPLE_RATE_HZ)  # 10000 samples

# --- PWM edge exclusion ---
# Samples straight after a LED on<->off transition are contaminated by the
# TIA/2nd-stage amplifier settling and by the switching transient itself.
# Excluding a small guard band around every transition keeps only "settled"
# samples in the on/off averages, which removes a systematic, non-random
# error the old code was folding straight into delta_v every single frame.
PWM_SETTLE_SAMPLES = 40  # ~0.2 ms guard band at 200 kSPS

# --- Outlier rejection + smoothing on the derived delta_v signal ---
# A median filter over the last few *measurements* rejects one-off spikes
# (a bad packet, an EMI hit) without touching genuine fast breath changes.
# The exponential filter that follows is time-constant based (not frame
# based) so it behaves the same regardless of GUI refresh rate, and it is
# applied to delta_v *before* the Beer-Lambert log so noise isn't distorted
# by that nonlinearity before being cleaned up.
DELTA_V_MEDIAN_LEN = 5
DEFAULT_SMOOTHING_TAU_S = 0.30
MIN_SMOOTHING_TAU_S = 0.02
MAX_SMOOTHING_TAU_S = 2.00

# --- TIA / analogue front-end model (see Figure 5.2 schematic) ---
TIA_RF_OHMS = 120e3               # Rf1, transimpedance feedback resistor
STAGE2_GAIN = 1 + (240e3 / 1e3)   # Av = 1 + Rf2/Rf3 = 241
TOTAL_TRANSIMPEDANCE = TIA_RF_OHMS * STAGE2_GAIN  # V per A

# Photosensitivity temperature coefficient of the InAsSb detector
ALPHA_PD_PER_DEGC = -0.001
T0_REF_DEGC = 25.0

# --- Detector responsivity, used to reverse photocurrent -> incident optical power ---
DETECTOR_RESPONSIVITY_A_PER_W = 4.5e-3  # S = 4.5 mA/W typ. at lambda_p

# --- Physical sanity clamp on photocurrent ---
MAX_PHYSICAL_PHOTOCURRENT_NA = 2.0

# --- NDIR CO2 (Beer-Lambert) calibration defaults ---
DEFAULT_PATH_LENGTH_CM = 3
DEFAULT_ABS_COEFF_PER_PCT_CM = 0.10

# --- Fixed display window (seconds) for both plots ---
DISPLAY_WINDOW_S = 5.0

# --- Breath-control status thresholds (heuristic, tune to taste) ---
CO2_IDLE_PCT = 0.20
CO2_SUSTAIN_MIN_PCT = 1.00
CO2_OVERBLOW_PCT = 5.00
SLOPE_RISE_THRESH_PCT_S = 3.0
SLOPE_FALL_THRESH_PCT_S = -3.0
SLOPE_STEADY_BAND_PCT_S = 1.0
LEAK_STD_THRESH_PCT = 0.40
STATUS_WINDOW_S = 0.75

# --- Shared state (guarded by `lock`) ---
lock = threading.Lock()
running = True

rolling_volts = np.array([], dtype=np.float32)
rolling_pwm = np.array([], dtype=np.uint8)
latest_temp = None

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


class SignalConditioner:
    """
    Two-stage filter for a noisy scalar measurement stream:

    1. Rolling median over the last `median_len` raw measurements -- kills
       one-off spikes/outliers without lagging genuine step changes the way
       a mean filter would.
    2. First-order exponential (low-pass) filter with a real *time constant*
       (not a fixed number of frames), so it behaves consistently no matter
       how fast update() is actually called.
    """

    def __init__(self, median_len=DELTA_V_MEDIAN_LEN, tau_s=DEFAULT_SMOOTHING_TAU_S):
        self.median_len = median_len
        self.tau_s = tau_s
        self._raw_history = deque(maxlen=median_len)
        self._filtered_value = None
        self._last_t = None

    def set_tau(self, tau_s):
        self.tau_s = float(np.clip(tau_s, MIN_SMOOTHING_TAU_S, MAX_SMOOTHING_TAU_S))

    def reset(self):
        self._raw_history.clear()
        self._filtered_value = None
        self._last_t = None

    def update(self, value, t):
        """Feed a new raw measurement (or None if unavailable this tick)."""
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
    global rolling_volts, rolling_pwm, latest_temp

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
             temperature, _pressure) = struct.unpack(HEADER_REST_FMT, rest)

            if samples != EXPECTED_SAMPLES:
                continue

            adc_u16 = np.frombuffer(raw, dtype="<u2").copy()
            if checksum_u16(adc_u16) != checksum:
                continue

            pwm_state = ((adc_u16 >> 15) & 0x1).astype(np.uint8)
            adc_u16 = adc_u16 & 0x0FFF
            volts = adc_u16.astype(np.float32) * (VREF / ADC_MAX)

            now = time.time()
            get_session_start_time(now)

            with lock:
                rolling_volts = np.concatenate((rolling_volts, volts))
                rolling_pwm = np.concatenate((rolling_pwm, pwm_state))
                if len(rolling_volts) > ROLLING_ADC_SAMPLES:
                    rolling_volts = rolling_volts[-ROLLING_ADC_SAMPLES:]
                    rolling_pwm = rolling_pwm[-ROLLING_ADC_SAMPLES:]

                latest_temp = temperature

    finally:
        sock.close()


class SimpleCO2Window(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Trombone Mouthpiece CO2 Monitor")

        # --- LED on/off average + photocurrent history ---
        self.led_t = deque(maxlen=ROLLING_LED_POINTS)
        self.led_on_v = deque(maxlen=ROLLING_LED_POINTS)
        self.led_off_v = deque(maxlen=ROLLING_LED_POINTS)

        # Conditions raw delta_v (LED-on minus LED-off volts) before it ever
        # reaches the Beer-Lambert log -- see SignalConditioner docstring.
        self.delta_v_filter = SignalConditioner(tau_s=DEFAULT_SMOOTHING_TAU_S)
        self.latest_power_uw = None

        # --- CO2 / baseline state ---
        self.baseline_power_uw = None
        self.co2_path_length_cm = DEFAULT_PATH_LENGTH_CM
        self.co2_abs_coeff = DEFAULT_ABS_COEFF_PER_PCT_CM
        self.co2_start_time = None
        self.co2_t = deque(maxlen=ROLLING_CO2_POINTS)
        self.co2_pct = deque(maxlen=ROLLING_CO2_POINTS)

        # =========================================================
        # Single-column layout: CO2 readout on top, deltaV plot below
        # =========================================================
        root = QtWidgets.QVBoxLayout(self)

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
        self.path_length_spin.valueChanged.connect(self.update_path_length)
        calib_row1.addWidget(self.path_length_spin)

        calib_row1.addWidget(QtWidgets.QLabel("Abs. coeff (per %CO2\u00b7cm)"))
        self.abs_coeff_spin = QtWidgets.QDoubleSpinBox()
        self.abs_coeff_spin.setRange(0.01, 10.0)
        self.abs_coeff_spin.setSingleStep(0.05)
        self.abs_coeff_spin.setValue(self.co2_abs_coeff)
        self.abs_coeff_spin.valueChanged.connect(self.update_abs_coeff)
        calib_row1.addWidget(self.abs_coeff_spin)

        calib_row1.addWidget(QtWidgets.QLabel("Response time (ms)"))
        self.smoothing_spin = QtWidgets.QSpinBox()
        self.smoothing_spin.setRange(int(MIN_SMOOTHING_TAU_S * 1000), int(MAX_SMOOTHING_TAU_S * 1000))
        self.smoothing_spin.setSingleStep(10)
        self.smoothing_spin.setValue(int(DEFAULT_SMOOTHING_TAU_S * 1000))
        self.smoothing_spin.setToolTip(
            "Filter time constant applied to \u0394V before the CO2 calculation.\n"
            "Lower = snappier but noisier. Higher = smoother but slower to react."
        )
        self.smoothing_spin.valueChanged.connect(self.update_smoothing_tau)
        calib_row1.addWidget(self.smoothing_spin)

        calib_row1.addStretch(1)
        status_col.addLayout(calib_row1)

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
        self.co2_plot_widget.setMinimumHeight(300)
        self.co2_plot_widget.setXRange(0, DISPLAY_WINDOW_S, padding=0)
        self.co2_plot_widget.getViewBox().disableAutoRange(axis=pg.ViewBox.XAxis)
        co2_layout.addWidget(self.co2_plot_widget, stretch=1)

        root.addWidget(co2_group, stretch=1)

        # =========================================================
        # deltaV plot: LED-on (pink) vs LED-off (white) photodiode voltage
        # =========================================================
        deltav_group = QtWidgets.QGroupBox("Photodiode Voltage - LED On/Off (\u0394V, 50 ms avg, edge-excluded)")
        deltav_layout = QtWidgets.QVBoxLayout(deltav_group)

        self.deltav_plot_widget = pg.PlotWidget(title="LED On (pink) / LED Off (white)")
        self.deltav_plot_widget.setLabel("bottom", "Time", units="s")
        self.deltav_plot_widget.setLabel("left", "Voltage", units="V")
        self.deltav_plot_widget.showGrid(x=True, y=True)
        self.deltav_plot_widget.setBackground("k")
        self.led_on_curve = self.deltav_plot_widget.plot(
            pen=pg.mkPen(width=2, color="#ff69b4"), name="LED On"
        )
        self.led_off_curve = self.deltav_plot_widget.plot(
            pen=pg.mkPen(width=2, color="#ffffff"), name="LED Off"
        )
        self.deltav_plot_widget.addLegend()
        self.deltav_plot_widget.setMinimumHeight(300)
        self.deltav_plot_widget.setXRange(0, DISPLAY_WINDOW_S, padding=0)
        self.deltav_plot_widget.getViewBox().disableAutoRange(axis=pg.ViewBox.XAxis)
        deltav_layout.addWidget(self.deltav_plot_widget, stretch=1)

        root.addWidget(deltav_group, stretch=1)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(33)

    # --- UI callbacks: CO2 calibration / baseline ---
    def update_path_length(self, value):
        self.co2_path_length_cm = float(value)

    def update_abs_coeff(self, value):
        self.co2_abs_coeff = float(value)

    def update_smoothing_tau(self, value_ms):
        self.delta_v_filter.set_tau(value_ms / 1000.0)

    def capture_baseline(self):
        if self.latest_power_uw is None:
            QtWidgets.QMessageBox.warning(
                self, "Capture Baseline",
                "No photocurrent data yet -- wait for the LED/detector to start streaming."
            )
            return

        self.baseline_power_uw = self.latest_power_uw

        baseline_text = f"Baseline: {self.baseline_power_uw:.4f} \u00b5W (captured on air)"
        self.baseline_label.setText(baseline_text)

        # Zero the CO2 timeline/history so the main display starts fresh
        # relative to this new reference.
        self.co2_start_time = time.time()
        self.co2_t.clear()
        self.co2_pct.clear()
        self.co2_curve.setData([], [])
        self.delta_v_filter.reset()

    # --- LED on/off averaging + photocurrent + optical power + CO2 ---
    def compute_led_averages(self, volts_full, pwm_full):
        """
        Windowed average of ADC voltage, split by PWM (LED) state, with a
        guard band excluded around every on<->off transition so amplifier
        settling / switching transients never contaminate the average.
        """
        if len(volts_full) == 0:
            return None, None

        n = min(len(volts_full), LED_AVG_WINDOW_SAMPLES)
        window_v = volts_full[-n:]
        window_pwm = pwm_full[-n:].astype(np.int8)

        edge_mask = np.zeros(len(window_pwm), dtype=bool)
        transitions = np.flatnonzero(np.diff(window_pwm) != 0)
        for idx in transitions:
            lo = max(0, idx - PWM_SETTLE_SAMPLES)
            hi = min(len(window_pwm), idx + PWM_SETTLE_SAMPLES + 1)
            edge_mask[lo:hi] = True

        on_mask = (window_pwm == 1) & ~edge_mask
        off_mask = (window_pwm == 0) & ~edge_mask

        led_on_avg = float(window_v[on_mask].mean()) if np.any(on_mask) else None
        led_off_avg = float(window_v[off_mask].mean()) if np.any(off_mask) else None
        return led_on_avg, led_off_avg

    def compute_photocurrent_na(self, delta_v, temperature_c):
        """
        Photocurrent from the TIA + 2nd stage gain, temperature-compensated
        back to a 25 degC-equivalent reading, with an out-of-range reading
        rejected rather than corrupting the CO2 chain. `delta_v` is expected
        to already be filtered (see SignalConditioner).
        """
        if delta_v is None:
            return None

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

    def classify_breath_status(self, co2_pct, slope, std_dev):
        """Heuristic breath-control classifier based on CO2 level/slope/stability."""
        if co2_pct is None:
            return "No Baseline / Awaiting Capture", "#888888"

        if co2_pct <= CO2_IDLE_PCT:
            return "Not Playing (Resting)", "#666666"

        if co2_pct >= CO2_OVERBLOW_PCT and slope >= 0:
            return "High CO2 - Possible Over-blowing", "#ff6600"

        if slope >= SLOPE_RISE_THRESH_PCT_S:
            return "Attack - Breath Onset", "#3399ff"

        if slope <= SLOPE_FALL_THRESH_PCT_S:
            return "Release - Breath Ending", "#9966ff"

        if (co2_pct >= CO2_SUSTAIN_MIN_PCT and std_dev <= LEAK_STD_THRESH_PCT
                and abs(slope) < SLOPE_STEADY_BAND_PCT_S):
            return "Sustained Airflow - Good Support", "#33cc33"

        if co2_pct > CO2_IDLE_PCT and std_dev > LEAK_STD_THRESH_PCT:
            return "Unstable - Possible Embouchure Leak", "#ffcc00"

        return "Transitional", "#aaaaaa"

    # --- Plot refresh ---
    def update_plot(self):
        with lock:
            volts_full = rolling_volts.copy()
            pwm_full = rolling_pwm.copy()
            temperature_now = latest_temp

        now = time.time()
        t0 = get_session_start_time(now)

        # ---- Edge-excluded windowed LED on/off average (raw, for the plot) ----
        led_on_avg, led_off_avg = self.compute_led_averages(volts_full, pwm_full)

        if led_on_avg is not None and led_off_avg is not None:
            self.led_t.append(now - t0)
            self.led_on_v.append(led_on_avg)
            self.led_off_v.append(led_off_avg)

            self.led_on_curve.setData(list(self.led_t), list(self.led_on_v))
            self.led_off_curve.setData(list(self.led_t), list(self.led_off_v))

            t_latest = self.led_t[-1]
            self.deltav_plot_widget.setXRange(
                t_latest - DISPLAY_WINDOW_S, t_latest, padding=0
            )

            raw_delta_v = led_on_avg - led_off_avg
        else:
            raw_delta_v = None

        # ---- Median + time-constant EMA filter, applied BEFORE the Beer-
        # Lambert log so noise is cleaned up before that nonlinearity can
        # distort/amplify it -- this is the main fix for the jitter. ----
        filtered_delta_v = self.delta_v_filter.update(raw_delta_v, now)

        i_pd_na = self.compute_photocurrent_na(filtered_delta_v, temperature_now)
        p_uw = self.photocurrent_to_optical_power_uw(i_pd_na)
        if p_uw is not None:
            self.latest_power_uw = p_uw

        # ---- CO2 % (needs a captured baseline) ----
        if self.baseline_power_uw is not None and self.latest_power_uw is not None:
            co2 = self.compute_co2_percent(self.latest_power_uw)

            if co2 is not None:
                if self.co2_start_time is None:
                    self.co2_start_time = now
                self.co2_t.append(now - self.co2_start_time)
                self.co2_pct.append(co2)
                self.co2_curve.setData(list(self.co2_t), list(self.co2_pct))

                t_latest_co2 = self.co2_t[-1]
                self.co2_plot_widget.setXRange(
                    t_latest_co2 - DISPLAY_WINDOW_S, t_latest_co2, padding=0
                )

                slope, std_dev = self.compute_slope_and_std()
                status_text, status_color = self.classify_breath_status(co2, slope, std_dev)

                self.co2_digital_label.setText(f"{co2:5.2f} %")
                self.breath_status_label.setText(status_text)
                self.breath_status_label.setStyleSheet(
                    f"background-color: {status_color}; color: white; padding: 6px; border-radius: 6px;"
                )
        else:
            self.co2_digital_label.setText("--.-- %")


def main():
    global running

    app = QtWidgets.QApplication(sys.argv)

    reader = threading.Thread(target=udp_thread, daemon=True)
    reader.start()

    window = SimpleCO2Window()
    window.resize(900, 900)
    window.show()

    def cleanup():
        global running
        running = False
        reader.join(timeout=1)

    app.aboutToQuit.connect(cleanup)

    sys.exit(app.exec())


if __name__ == "__main__":
    main()