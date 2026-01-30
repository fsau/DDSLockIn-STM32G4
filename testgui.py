import sys
import time
import numpy as np
import serial
import serial.tools.list_ports
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore
from scipy import optimize  # For least squares

# ------------------------------------------------------------
# Same helper as Octave
# ------------------------------------------------------------
def fs(f):
    return int(round(f * (2**28) / 25e6))

# ------------------------------------------------------------
# GUI
# ------------------------------------------------------------
class SweepGUI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Serial Sweep")
        self.resize(1000, 500)

        # ---------------- Serial ----------------
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            raise RuntimeError("No serial ports found")

        self.ser = serial.Serial(
            ports[-1].device,
            baudrate=115200,   # adjust if needed
            timeout=0.2
        )

        self.flush_input()

        # ---------------- UI ----------------
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)

        # ---- Plot ----
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot = self.plot_widget.addPlot(title="Oscilloscope")
        self.plot.showGrid(x=True, y=True)
        self.plot.setLabel('left', 'Amplitude', units='V')
        self.plot.setLabel('bottom', 'Time', units='us')
        
        self.plot_widget.nextRow()
        
        # Create plot for impedance magnitude (log scale)
        self.plot2 = self.plot_widget.addPlot(title="Impedance Analyzer")
        self.plot2.showGrid(x=True, y=True)
        
        # Setup log scale for impedance - FIXED: use correct method
        self.plot2.setLogMode(y=True, x=False)
        self.plot2.setLabel('left', 'Impedance', units='Ω')
        self.plot2.setLabel('bottom', 'Frequency', units='Hz')
        
        # Set reasonable default ranges
        self.plot2.setYRange(1, 10)  # Ω range
        self.plot2.setXRange(32700, 32800)  # Hz range
        
        # Add right axis for phase using ViewBox method
        self.plot2.showAxis('right')
        right_axis = self.plot2.getAxis('right')
        right_axis.setLabel('Phase', units='°', color='b')
        right_axis.setPen(color='b')
        right_axis.setTickPen(color='r')
        right_axis.setTextPen(color='b')
        
        # Create separate ViewBox for phase
        self.phase_vb = pg.ViewBox()
        self.plot2.scene().addItem(self.phase_vb)
        self.plot2.getAxis('right').linkToView(self.phase_vb)
        self.phase_vb.setXLink(self.plot2)
        self.phase_vb.setYRange(-180, 180)
        
        # Update connection
        self.update_views()

        self.curve0 = self.plot.plot(pen='y', name="CH0")
        self.curve1 = self.plot.plot(pen='c', name="CH1")
        
        # Impedance magnitude curve (left axis, log scale)
        self.curve_mag = self.plot2.plot(pen='r', symbol='o', symbolPen='r', 
                                         symbolBrush='r', symbolSize=5, name="|Z|")
        
        # Phase curve (right axis) - add to phase ViewBox
        self.curve_phase = pg.PlotCurveItem(pen='b', symbol='s', symbolPen='b',
                                           symbolBrush='b', symbolSize=5, name="Phase")
        self.phase_vb.addItem(self.curve_phase)

        layout.addWidget(self.plot_widget, stretch=3)

        # ---- Controls ----
        controls = QtWidgets.QWidget()
        form = QtWidgets.QFormLayout(controls)
        layout.addWidget(controls, stretch=1)

        self.fi_box = QtWidgets.QSpinBox()
        self.fi_box.setRange(1, 1_000_000)
        self.fi_box.setValue(32700)

        self.fo_box = QtWidgets.QSpinBox()
        self.fo_box.setRange(1, 1_000_000)
        self.fo_box.setValue(32800)
        
        # Known test resistor value (for calibration)
        self.r_ref_label = QtWidgets.QLabel("R_ref (Ω):")
        self.r_ref_box = QtWidgets.QDoubleSpinBox()
        self.r_ref_box.setRange(0.1, 100000)
        self.r_ref_box.setValue(1000.0)
        self.r_ref_box.setDecimals(1)

        # Sampling parameters
        self.sample_rate_label = QtWidgets.QLabel("Sample Rate (kHz):")
        self.sample_rate_box = QtWidgets.QDoubleSpinBox()
        self.sample_rate_box.setRange(1000, 1000000)
        self.sample_rate_box.setValue(2833.3333)
        self.sample_rate_box.setDecimals(3)

        self.start_btn = QtWidgets.QPushButton("Start / Update")
        self.stop_btn = QtWidgets.QPushButton("Stop")
        self.clear_btn = QtWidgets.QPushButton("Clear Data")
        self.status = QtWidgets.QLabel("Idle")
        
        # Legend
        self.legend = pg.LegendItem(offset=(70, 30))
        self.legend.setParentItem(self.plot2)
        self.legend.addItem(self.curve_mag, 'Impedance (Ω)')
        self.legend.addItem(self.curve_phase, 'Phase (°)')

        form.addRow("fi [Hz]:", self.fi_box)
        form.addRow("fo [Hz]:", self.fo_box)
        form.addRow(self.r_ref_label, self.r_ref_box)
        form.addRow(self.sample_rate_label, self.sample_rate_box)
        form.addRow(self.start_btn)
        form.addRow(self.stop_btn)
        form.addRow(self.clear_btn)
        form.addRow("Status:", self.status)

        # ---------------- Sweep state ----------------
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.sweep_step)

        self.fw = 0
        self.fw_end = 0
        
        # Store sweep data
        self.frequencies = []
        self.impedance_magnitudes = []
        self.phases = []

        self.start_btn.clicked.connect(self.start_sweep)
        self.stop_btn.clicked.connect(self.stop_sweep)
        self.clear_btn.clicked.connect(self.clear_data)
        
        # Connect plot resize
        self.plot2.sigRangeChanged.connect(self.update_views)

    # --------------------------------------------------------
    def update_views(self):
        """Update phase ViewBox geometry when main plot changes"""
        self.phase_vb.setGeometry(self.plot2.vb.sceneBoundingRect())
        self.phase_vb.linkedViewChanged(self.plot2.vb, self.phase_vb.XAxis)

    # --------------------------------------------------------
    def flush_input(self):
        self.ser.reset_input_buffer()
        time.sleep(0.05)
        while self.ser.in_waiting:
            self.ser.read(self.ser.in_waiting)
            time.sleep(0.01)

    # --------------------------------------------------------
    def start_sweep(self):
        fi = self.fi_box.value()
        fo = self.fo_box.value()

        if fo < fi:
            self.status.setText("fo < fi")
            return

        self.flush_input()

        self.fw = fs(fi)
        self.fw_end = fs(fo)
        
        # Clear previous data
        self.frequencies.clear()
        self.impedance_magnitudes.clear()
        self.phases.clear()
        self.plot2.setXRange(fi,fo)

        self.status.setText("Sweeping...")
        self.timer.start(1)   # 1 ms per step

    # --------------------------------------------------------
    def stop_sweep(self):
        self.timer.stop()
        self.status.setText("Stopped")
        
    # --------------------------------------------------------
    def clear_data(self):
        """Clear all stored sweep data"""
        self.frequencies.clear()
        self.impedance_magnitudes.clear()
        self.phases.clear()
        self.curve_mag.setData([], [])
        self.curve_phase.setData([], [])
        self.status.setText("Data cleared")

    # --------------------------------------------------------
    def calculate_impedance_lsq(self, ch0, ch1, freq_hz):
        """
        Calculate impedance using least squares sine fit.
        ch0: voltage across reference resistor
        ch1: voltage across DUT
        freq_hz: known excitation frequency
        Returns: impedance magnitude, phase (degrees)
        """
        R_ref = self.r_ref_box.value()
        sample_rate = self.sample_rate_box.value()*1000
        n_samples = len(ch0)
        
        # Time vector
        t = np.arange(n_samples) / sample_rate
        
        # Angular frequency
        omega = 2 * np.pi * freq_hz
        
        # Create design matrix for least squares fit
        # Model: A*cos(ωt) + B*sin(ωt) + C
        X = np.column_stack([
            np.cos(omega * t),
            np.sin(omega * t),
            np.ones_like(t)
        ])
        
        # Fit ch0 (reference)
        coeffs_ch0, *_ = np.linalg.lstsq(X, ch0, rcond=None)
        A0, B0, C0 = coeffs_ch0
        
        # Fit ch1 (DUT)
        coeffs_ch1, *_ = np.linalg.lstsq(X, ch1, rcond=None)
        A1, B1, C1 = coeffs_ch1
        
        # Convert to complex amplitude
        # V = A - jB  (since cos(ωt) + j sin(ωt) = e^{jωt})
        V_ref_complex = A0 - 1j * B0
        V_dut_complex = A1 - 1j * B1
        
        # Calculate impedance
        # I_ref = V_ref / R_ref
        # Z = V_dut / I_ref = V_dut * R_ref / V_ref
        if abs(V_ref_complex) > 1e-10:  # Avoid division by zero
            Z_complex = V_dut_complex * R_ref / V_ref_complex
            Z_mag = abs(Z_complex)
            Z_phase = np.angle(Z_complex, deg=True)
        else:
            Z_mag = 0
            Z_phase = 0
        
        # Alternative simple amplitude ratio (fallback)
        if Z_mag == 0 or not np.isfinite(Z_mag):
            # Use amplitude ratio method
            amp_ch0 = np.sqrt(A0**2 + B0**2)
            amp_ch1 = np.sqrt(A1**2 + B1**2)
            if amp_ch0 > 0:
                Z_mag = (amp_ch1 / amp_ch0) * R_ref
                # Approximate phase from phase difference
                phase0 = np.arctan2(-B0, A0)  # Note: -B because of our convention
                phase1 = np.arctan2(-B1, A1)
                Z_phase = np.degrees(phase1 - phase0)
            else:
                Z_mag = 0
                Z_phase = 0
        
        # Normalize phase to [-180, 180]
        while Z_phase > 180:
            Z_phase -= 360
        while Z_phase < -180:
            Z_phase += 360
        
        return Z_mag, Z_phase

    # --------------------------------------------------------
    def sweep_step(self):
        if self.fw > self.fw_end:
            self.timer.stop()
            self.status.setText("Done")
            return

        # ----- send frequency -----
        cmd = f"ff{self.fw}a\r\n".encode()
        self.ser.write(cmd)
        time.sleep(0.0001)
        self.ser.reset_output_buffer()

        # ----- trigger measurement -----
        self.ser.write(b"m")

        raw = self.ser.read(1024 * 4)

        if len(raw) != 1024 * 4:
            self.timer.stop()
            self.status.setText("Short read")
            return

        # ----- data conversion (bit-exact) -----
        raw_u8 = np.frombuffer(raw, dtype=np.uint8)
        data = raw_u8[0::2].astype(np.int16) + \
               (raw_u8[1::2].astype(np.int16) << 8)

        if data.size != 2048:
            self.timer.stop()
            self.status.setText("Frame error")
            return

        half = data.size // 2
        ch0 = (data[:half]-2048)*3.3/4096
        ch1 = (data[half:]-2048)*3.3/4096

        # ----- update scope plot -----
        self.curve0.setData(ch0)
        self.curve1.setData(ch1)

        # ----- get current frequency -----
        freq_hz = self.fw / (2**28) * 25e6
        
        # ----- calculate impedance using least squares -----
        Z_mag, Z_phase = self.calculate_impedance_lsq(ch0, ch1, freq_hz)
        
        # ----- store data point -----
        self.frequencies.append(freq_hz)
        self.impedance_magnitudes.append(Z_mag)
        self.phases.append(Z_phase)
        
        # ----- update analyzer plots -----
        if len(self.frequencies) > 0:
            # Convert to numpy arrays for plotting
            freqs_np = np.array(self.frequencies)
            mags_np = np.array(self.impedance_magnitudes)
            phases_np = np.array(self.phases)
            
            # Filter out invalid values
            valid_idx = (mags_np > 0) & np.isfinite(mags_np) & np.isfinite(phases_np)
            
            if np.any(valid_idx):
                self.curve_mag.setData(freqs_np[valid_idx], mags_np[valid_idx])
                self.curve_phase.setData(freqs_np[valid_idx], phases_np[valid_idx])
        
        self.status.setText(f"f = {freq_hz:.1f} Hz, |Z| = {Z_mag:.2f} Ω, φ = {Z_phase:.1f}°")

        self.fw += 1


# ------------------------------------------------------------
# Run
# ------------------------------------------------------------
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    
    # Set dark theme for better visibility
    app.setStyle('Fusion')
    
    gui = SweepGUI()
    gui.show()
    sys.exit(app.exec())