import sys
import time
import numpy as np
import serial
import serial.tools.list_ports
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore
from scipy import optimize  # For least squares
from PyQt5.QtCore import Qt
import os
from datetime import datetime
import re

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
        self.plot.setLabel('bottom', 'Time', units='μs')
        self.plot.getAxis('bottom').enableAutoSIPrefix(False)
        
        self.plot_widget.nextRow()
        
        # Create plot for impedance magnitude (log scale)
        self.plot2 = self.plot_widget.addPlot(title="Impedance Analyzer")
        self.plot2.showGrid(x=True, y=True)
        
        # Setup log scale for impedance - FIXED: use correct method
        self.plot2.setLogMode(y=True, x=False)
        self.plot2.setLabel('left', 'Impedance', units='Ω')
        self.plot2.setLabel('bottom', 'Frequency', units='Hz')
        self.plot2.getAxis('left').enableAutoSIPrefix(False)
        
        # Set reasonable default ranges
        self.plot2.setYRange(4, 8)  # Ω range
        self.plot2.setXRange(32720, 32800)  # Hz range
        
        # Add right axis for phase using ViewBox method
        self.plot2.showAxis('right')
        right_axis = self.plot2.getAxis('right')
        right_axis.setLogMode(y=False, x=False)
        right_axis.setLabel('Phase', units='°', color='b')
        right_axis.setTextPen()
        
        # Create separate ViewBox for phase
        self.phase_vb = pg.ViewBox()
        self.plot2.scene().addItem(self.phase_vb)
        self.plot2.getAxis('right').linkToView(self.phase_vb)
        self.phase_vb.setXLink(self.plot2)
        self.phase_vb.setYRange(-100, 100)
        
        self.plot_widget.ci.layout.setRowStretchFactor(0, 3)  # Oscilloscope: 30%
        self.plot_widget.ci.layout.setRowStretchFactor(1, 7)  # Analyzer: 70%
        
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
        self.fi_box.setValue(32720)

        self.fo_box = QtWidgets.QSpinBox()
        self.fo_box.setRange(1, 1_000_000)
        self.fo_box.setValue(32800)
        
        # Delta fw per step (integer input)
        self.delta_fw_label = QtWidgets.QLabel("Δfw per step:")
        self.delta_fw_box = QtWidgets.QSpinBox()
        self.delta_fw_box.setRange(1, 1000)
        self.delta_fw_box.setValue(1)
        
        # Feedback resistor value (for calibration)
        self.r_ref_label = QtWidgets.QLabel("R_ref (kΩ):")
        self.r_ref_box = QtWidgets.QDoubleSpinBox()
        self.r_ref_box.setRange(0.1, 100000)
        self.r_ref_box.setValue(2000.0)
        self.r_ref_box.setDecimals(1)

        # Feedback capacitance field
        self.c_ref_label = QtWidgets.QLabel("C_ref (pF):")
        self.c_ref_box = QtWidgets.QDoubleSpinBox()
        self.c_ref_box.setRange(0.1, 10000)
        self.c_ref_box.setValue(10.5)
        self.c_ref_box.setDecimals(1)

        # Sampling parameters
        self.sample_rate_label = QtWidgets.QLabel("Sample Rate (kHz):")
        self.sample_rate_box = QtWidgets.QDoubleSpinBox()
        self.sample_rate_box.setRange(1000, 1000000)
        self.sample_rate_box.setValue(2833.3333)
        self.sample_rate_box.setDecimals(3)

        # Time delay before starting sweep (ms)
        self.start_delay_label = QtWidgets.QLabel("Start Delay (ms):")
        self.start_delay_box = QtWidgets.QDoubleSpinBox()
        self.start_delay_box.setRange(0, 10000)
        self.start_delay_box.setValue(0.0)
        self.start_delay_box.setDecimals(1)

        # Delay between samples (ms)
        self.sample_delay_label = QtWidgets.QLabel("Sample Delay (ms):")
        self.sample_delay_box = QtWidgets.QDoubleSpinBox()
        self.sample_delay_box.setRange(0, 10000)
        self.sample_delay_box.setValue(0.0)
        self.sample_delay_box.setDecimals(1)

        # Number of measurements to average for each point
        self.average_count_label = QtWidgets.QLabel("Average Count:")
        self.average_count_box = QtWidgets.QSpinBox()
        self.average_count_box.setRange(1, 100)
        self.average_count_box.setValue(1)

        self.start_btn = QtWidgets.QPushButton("Start / Update")
        self.stop_btn = QtWidgets.QPushButton("Stop")
        self.clear_btn = QtWidgets.QPushButton("Clear Data")
        
        # CSV Save Button and fields
        self.save_btn = QtWidgets.QPushButton("Save to CSV")
        
        # Filename field (with auto-numbering)
        self.filename_label = QtWidgets.QLabel("Filename:")
        self.filename_edit = QtWidgets.QLineEdit()
        self.filename_edit.setText("sweep_data_001.csv")
        
        # Notes field
        self.notes_label = QtWidgets.QLabel("Notes:")
        self.notes_edit = QtWidgets.QTextEdit()
        self.notes_edit.setMaximumHeight(60)
        self.notes_edit.setPlaceholderText("Enter notes for this measurement...")
        
        # Create a text edit widget for multi-line status with right alignment
        self.status = QtWidgets.QTextEdit()
        self.status.setReadOnly(True)
        self.status.setMinimumHeight(110)
        self.status.setText("Idle")
        
        # Set right alignment
        alignment = self.status.alignment()
        self.status.setAlignment(alignment)
        
        # Legend
        self.legend = pg.LegendItem(offset=(70, 30))
        self.legend.setParentItem(self.plot2)
        self.legend.addItem(self.curve_mag, 'Impedance (Ω)')
        self.legend.addItem(self.curve_phase, 'Phase (°)')

        form.addRow("fi [Hz]:", self.fi_box)
        form.addRow("fo [Hz]:", self.fo_box)
        form.addRow(self.delta_fw_label, self.delta_fw_box)
        form.addRow(self.r_ref_label, self.r_ref_box)
        form.addRow(self.c_ref_label, self.c_ref_box)
        form.addRow(self.sample_rate_label, self.sample_rate_box)
        form.addRow(self.start_delay_label, self.start_delay_box)
        form.addRow(self.sample_delay_label, self.sample_delay_box)
        form.addRow(self.average_count_label, self.average_count_box)
        form.addRow(self.start_btn)
        form.addRow(self.stop_btn)
        form.addRow(self.clear_btn)
        
        # Add separator
        form.addRow(QtWidgets.QLabel("--- Save Options ---"))
        form.addRow(self.filename_label, self.filename_edit)
        form.addRow(self.notes_label, self.notes_edit)
        form.addRow(self.save_btn)
        
        spacer = QtWidgets.QSpacerItem(20, 40)
        form.addItem(spacer)
        form.addRow(self.status)

        # ---------------- Sweep state ----------------
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.sweep_step)

        self.fw = 0
        self.fw_end = 0
        self.sweep_started = False
        self.waiting_for_start = False
        
        # Store sweep data
        self.frequencies = []
        self.impedance_magnitudes = []
        self.phases = []
        
        # Store detailed data for CSV export
        self.detailed_data = []  # List of dictionaries with all measured data

        self.start_btn.clicked.connect(self.start_sweep)
        self.stop_btn.clicked.connect(self.stop_sweep)
        self.clear_btn.clicked.connect(self.clear_data)
        self.save_btn.clicked.connect(self.save_to_csv)
        
        # Connect plot resize
        self.plot2.sigRangeChanged.connect(self.update_views)
        
        # Initialize filename with suggestion
        self.generate_new_filename()

    # --------------------------------------------------------
    def generate_new_filename(self):
        """Generate a new filename with auto-numbering based on existing files"""
        current_filename = self.filename_edit.text().strip()
        
        # Extract base name and existing counter
        match = re.match(r'^(.*?)(?:_(\d{3}))?\.csv$', current_filename)
        
        if match:
            base_name = match.group(1)
            existing_counter = match.group(2)
            
            if existing_counter:
                # Start from existing counter + 1
                counter = int(existing_counter) + 1
                base_name = base_name.rstrip('_')  # Remove trailing underscore if present
            else:
                # No counter found, start from 001
                counter = 1
        else:
            # No .csv extension or pattern doesn't match
            base_name = re.sub(r'\.csv$', '', current_filename)
            if not base_name:
                base_name = "sweep_data"
            counter = 1
        
        # Find the next available number if file exists
        while os.path.exists(f"{base_name}_{counter:03d}.csv"):
            counter += 1
        
        # Set the filename without triggering textChanged
        self.filename_edit.blockSignals(True)
        self.filename_edit.setText(f"{base_name}_{counter:03d}.csv")
        self.filename_edit.blockSignals(False)

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
        self.detailed_data.clear()
        self.plot2.setXRange(fi,fo)

        self.sweep_started = False
        self.waiting_for_start = True
        
        cmd = f"ff{self.fw}a\r\n".encode()
        self.ser.write(cmd)
        
        # Apply start delay before beginning sweep
        start_delay_ms = self.start_delay_box.value()
        if start_delay_ms > 0:
            self.status.setText(f"Waiting {start_delay_ms}ms before starting...")
            QtCore.QTimer.singleShot(int(start_delay_ms), self.begin_sweep)
        else:
            self.begin_sweep()

    # --------------------------------------------------------
    def begin_sweep(self):
        """Start the actual sweep after initial delay"""
        self.waiting_for_start = False
        self.sweep_started = True
        self.status.setText("Sweeping...")
        self.timer.start(1)  # Start with minimal timer interval

    # --------------------------------------------------------
    def stop_sweep(self):
        self.timer.stop()
        self.sweep_started = False
        self.waiting_for_start = False
        self.status.setText("Stopped")
        
    # --------------------------------------------------------
    def clear_data(self):
        """Clear all stored sweep data"""
        self.frequencies.clear()
        self.impedance_magnitudes.clear()
        self.phases.clear()
        self.detailed_data.clear()
        self.curve_mag.setData([], [])
        self.curve_phase.setData([], [])
        self.status.setText("Data cleared")
        # Generate a new filename for next save
        self.generate_new_filename()

    # --------------------------------------------------------
    def calculate_impedance_lsq(self, ch0, ch1, freq_hz):
        """
        Calculate impedance using least squares sine fit.
        ch0: voltage across DUT (V_dut)
        ch1: current through reference (I_ref) 
        freq_hz: known excitation frequency
        
        Impedance: Z_dut = V_dut / I_dut
        But I_dut = I_ref (series circuit)
        And V_ref = I_ref * Z_ref (parallel RC)
        
        So: Z_dut = V_dut / I_ref
        Where I_ref is complex current from CH1
        
        Returns: impedance magnitude, phase (degrees), phase of voltage channel for time alignment,
                 residuals for both channels, and DC levels, and all fitted parameters
        """
        R_ref = self.r_ref_box.value() * 1e3
        C_ref = self.c_ref_box.value() * 1e-12  # Convert pF to F
        sample_rate = self.sample_rate_box.value() * 1000
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
        
        # Fit ch0 (V_dut - voltage across DUT)
        coeffs_ch0, residuals_ch0, *_ = np.linalg.lstsq(X, ch0, rcond=None)
        A_v, B_v, C_v = coeffs_ch0
        
        # Fit ch1 (I_ref - current through reference)
        coeffs_ch1, residuals_ch1, *_ = np.linalg.lstsq(X, ch1, rcond=None)
        A_i, B_i, C_i = coeffs_ch1
        
        # Convert to complex amplitude
        # Signal = A*cos(ωt) + B*sin(ωt) = Re{(A - jB)e^{jωt}}
        V_complex = A_v - 1j * B_v
        I_complex = A_i - 1j * B_i
        
        # Calculate phase of voltage channel for time alignment
        voltage_phase = np.angle(I_complex)  # in radians
        
        # Calculate impedance of parallel RC reference
        # Z_ref = (R_ref || C_ref) = 1/(1/R_ref + jωC_ref)
        # Or equivalently: Z_ref = R_ref / (1 + jωR_refC_ref)
        Z_ref_complex = R_ref / (1 + 1j * omega * R_ref * C_ref)
        
        # Calculate DUT impedance: Z_dut = V_dut / I_ref
        if abs(I_complex) > 1e-10:  # Avoid division by zero
            Z_dut_complex = I_complex / V_complex * Z_ref_complex
            Z_mag = abs(Z_dut_complex)
            Z_phase = np.angle(-Z_dut_complex, deg=True)
        else:
            Z_mag = 0
            Z_phase = 0
        
        # Alternative simple amplitude ratio (fallback)
        if Z_mag == 0 or not np.isfinite(Z_mag):
            # Use amplitude ratio method
            amp_v = np.sqrt(A_v**2 + B_v**2)
            amp_i = np.sqrt(A_i**2 + B_i**2)
            if amp_i > 0:
                Z_mag = amp_v / amp_i
                # Approximate phase from phase difference
                phase_v = np.arctan2(-B_v, A_v)  # Note: -B because of our convention
                phase_i = np.arctan2(-B_i, A_i)
                Z_phase = np.degrees(phase_v - phase_i)
                voltage_phase = phase_v  # Use approximate phase
            else:
                Z_mag = 0
                Z_phase = 0
                voltage_phase = 0
        
        # Normalize phase to [-180, 180]
        while Z_phase > 180:
            Z_phase -= 360
        while Z_phase < -180:
            Z_phase += 360
        
        # Calculate RMS of residuals
        if len(residuals_ch0) > 0:
            rms_residuals_ch0 = np.sqrt(residuals_ch0[0] / n_samples)
        else:
            rms_residuals_ch0 = 0
            
        if len(residuals_ch1) > 0:
            rms_residuals_ch1 = np.sqrt(residuals_ch1[0] / n_samples)
        else:
            rms_residuals_ch1 = 0
        
        return (Z_mag, Z_phase, voltage_phase, rms_residuals_ch0, rms_residuals_ch1, 
                C_v, C_i, A_v, B_v, A_i, B_i, V_complex, I_complex)

    # --------------------------------------------------------
    def sweep_step(self):
        if self.waiting_for_start:
            return
            
        if not self.sweep_started:
            return
            
        if self.fw > self.fw_end:
            self.timer.stop()
            self.sweep_started = False
            self.status.setText("Done")
            # Generate new filename for next save after sweep completes
            self.generate_new_filename()
            return

        # Get number of measurements to average
        average_count = self.average_count_box.value()
        
        # Arrays to store measurements for averaging
        all_mag_measurements = []
        all_phase_measurements = []
        
        # Store detailed data for this frequency point
        freq_detailed_data = []
        
        # Store the last measurement data for plotting
        last_ch0 = None
        last_ch1 = None
        last_freq_hz = None
        last_voltage_phase = 0
        last_rms_residuals_ch0 = 0
        last_rms_residuals_ch1 = 0
        last_dc_ch0 = 0
        last_dc_ch1 = 0
        last_A_v = 0
        last_B_v = 0
        last_A_i = 0
        last_B_i = 0
        last_V_complex = 0+0j
        last_I_complex = 0+0j
        
        # Perform multiple measurements at this frequency
        for measurement_idx in range(average_count):
            # ----- send frequency -----
            cmd = f"ff{self.fw}a\r\n".encode()
            self.ser.write(cmd)
            time.sleep(0.0001)
            self.ser.reset_output_buffer()

            # Apply sample delay before measurement if set
            sample_delay_ms = self.sample_delay_box.value()
            if sample_delay_ms > 0:
                time.sleep(sample_delay_ms / 1000.0)

            # ----- trigger measurement -----
            self.ser.write(b"m")

            raw = self.ser.read(1024 * 4)

            if len(raw) != 1024 * 4:
                self.timer.stop()
                self.sweep_started = False
                self.status.setText("Short read")
                return

            # ----- data conversion (bit-exact) -----
            raw_u8 = np.frombuffer(raw, dtype=np.uint8)
            data = raw_u8[0::2].astype(np.int16) + \
                   (raw_u8[1::2].astype(np.int16) << 8)

            if data.size != 2048:
                self.timer.stop()
                self.sweep_started = False
                self.status.setText("Frame error")
                return

            half = data.size // 2
            ch0 = (data[:half]-2048)*3.3/4096  # V_dut (voltage across DUT)
            ch1 = (data[half:]-2048)*3.3/4096  # I_ref (current through reference)

            # Store last measurement for plotting
            last_ch0 = ch0
            last_ch1 = ch1
            
            # ----- get current frequency -----
            freq_hz = self.fw / (2**28) * 25e6
            last_freq_hz = freq_hz
            
            # ----- calculate impedance using least squares -----
            (Z_mag, Z_phase, voltage_phase, rms_residuals_ch0, rms_residuals_ch1, 
             dc_ch0, dc_ch1, A_v, B_v, A_i, B_i, V_complex, I_complex) = self.calculate_impedance_lsq(ch0, ch1, freq_hz)
            
            # Store all values
            last_voltage_phase = voltage_phase
            last_rms_residuals_ch0 = rms_residuals_ch0
            last_rms_residuals_ch1 = rms_residuals_ch1
            last_dc_ch0 = dc_ch0
            last_dc_ch1 = dc_ch1
            last_A_v = A_v
            last_B_v = B_v
            last_A_i = A_i
            last_B_i = B_i
            last_V_complex = V_complex
            last_I_complex = I_complex
            
            # Store this measurement in detailed data
            measurement_data = {
                'measurement_idx': measurement_idx,
                'freq_word': self.fw,
                'freq_hz': freq_hz,
                'Z_mag': Z_mag,
                'Z_phase': Z_phase,
                'V_complex_real': V_complex.real,
                'V_complex_imag': V_complex.imag,
                'I_complex_real': I_complex.real,
                'I_complex_imag': I_complex.imag,
                'A_v': A_v,
                'B_v': B_v,
                'A_i': A_i,
                'B_i': B_i,
                'dc_ch0': dc_ch0,
                'dc_ch1': dc_ch1,
                'rms_residuals_ch0': rms_residuals_ch0,
                'rms_residuals_ch1': rms_residuals_ch1,
                'voltage_phase_rad': voltage_phase
            }
            freq_detailed_data.append(measurement_data)
            
            # Store this measurement for averaging
            if Z_mag > 0 and np.isfinite(Z_mag) and np.isfinite(Z_phase):
                all_mag_measurements.append(Z_mag)
                all_phase_measurements.append(Z_phase)
                
            # Update status during averaging
            if average_count > 1:
                status_text = f"f = {freq_hz:.1f} Hz\nAvg {measurement_idx+1}/{average_count}"
                self.status.setText(status_text)
        
        # Calculate average of all valid measurements
        if all_mag_measurements:
            avg_mag = np.mean(all_mag_measurements)
            avg_phase = np.mean(all_phase_measurements)
        else:
            avg_mag = 0
            avg_phase = 0
        
        # ----- store averaged data point -----
        freq_hz = last_freq_hz
        self.frequencies.append(freq_hz)
        self.impedance_magnitudes.append(avg_mag)
        self.phases.append(avg_phase)
        
        # Store detailed data for this frequency point
        self.detailed_data.append({
            'freq_word': self.fw,
            'freq_hz': freq_hz,
            'Z_mag_avg': avg_mag,
            'Z_phase_avg': avg_phase,
            'measurements': freq_detailed_data,
            'avg_count': average_count
        })
        
        # ----- update oscilloscope plot with proper time alignment -----
        if last_ch0 is not None and last_ch1 is not None:
            sample_rate = self.sample_rate_box.value() * 1000  # Convert kHz to Hz
            n_samples = len(last_ch0)
            
            # Create time vector in microseconds
            t = np.arange(n_samples) / sample_rate * 1e6  # Convert to microseconds
            
            # Calculate time offset from voltage phase (convert phase to time)
            # Phase is in radians, convert to time delay: t_offset = phase / (2πf)
            if freq_hz > 0:
                t_offset = last_voltage_phase / (2 * np.pi * freq_hz) * 1e6  # Convert to microseconds
            else:
                t_offset = 0
            
            # Apply time offset to align waveforms
            t_aligned = t + t_offset - 2e6/(freq_hz)
            
            # Plot with aligned time axis
            self.curve0.setData(t_aligned, last_ch0)
            self.curve1.setData(t_aligned, last_ch1)
            
            # Set appropriate x-axis range (show about 2 periods)
            if freq_hz > 0:
                period_us = 1e6 / freq_hz
                self.plot.setXRange(0, 6 * period_us)
        
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
        
        # Format frequency with auto-scaling
        if freq_hz >= 1e6:
            freq_display = f"{freq_hz/1e6:.5f} MHz"
        elif freq_hz >= 1e3:
            freq_display = f"{freq_hz/1e3:.4f} kHz"
        else:
            freq_display = f"{freq_hz:.1f} Hz"
        
        # Format impedance with auto-scaling
        if avg_mag >= 1e6:
            z_display = f"{avg_mag/1e6:.2f} MΩ"
        elif avg_mag >= 1e3:
            z_display = f"{avg_mag/1e3:.2f} kΩ"
        else:
            z_display = f"{avg_mag:.2f} Ω"
        
        # Update status with detailed information
        if average_count > 1:
            status_text = (
                f"f = {freq_display}\n"
                f"|Z| = {z_display} (avg of {len(all_mag_measurements)})\n"
                f"φ = {avg_phase:.1f}°\n"
                f"Residuals: V={last_rms_residuals_ch0*1e3:.2f}mV, I={last_rms_residuals_ch1*1e3:.2f}mV\n"
                f"DC: V={last_dc_ch0*1e3:.1f}mV, I={last_dc_ch1*1e3:.1f}mV"
            )
        else:
            status_text = (
                f"f = {freq_display}\n"
                f"|Z| = {z_display}\n"
                f"φ = {avg_phase:.1f}°\n"
                f"Residuals: V={last_rms_residuals_ch0*1e3:.2f}mV, I={last_rms_residuals_ch1*1e3:.2f}mV\n"
                f"DC: V={last_dc_ch0*1e3:.1f}mV, I={last_dc_ch1*1e3:.1f}mV"
            )
        
        self.status.setText(status_text)
        # Scroll to the bottom to see the latest status
        self.status.verticalScrollBar().setValue(self.status.verticalScrollBar().maximum())

        # Increment fw by delta_fw instead of 1
        delta_fw = self.delta_fw_box.value()
        self.fw += delta_fw

    # --------------------------------------------------------
    def save_to_csv(self):
        """Save all sweep data to a CSV file"""
        if not self.detailed_data:
            self.status.setText("No data to save")
            return
        
        # Get filename from the edit field
        filename = self.filename_edit.text().strip()
        if not filename:
            # Use default if empty
            filename = "sweep_data_001.csv"
        
        # Remove any existing .csv extension and add it fresh
        filename = re.sub(r'\.csv$', '', filename) + '.csv'
        
        try:
            with open(filename, 'w', encoding='utf-8') as f:
                # Write comment with notes and parameters
                notes = self.notes_edit.toPlainText().strip()
                if notes:
                    f.write(f"# Notes: {notes}\n")
                
                # Write timestamp
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                f.write(f"# Timestamp: {timestamp}\n")
                
                # Write parameters
                f.write(f"# Parameters:\n")
                f.write(f"# fi = {self.fi_box.value()} Hz\n")
                f.write(f"# fo = {self.fo_box.value()} Hz\n")
                f.write(f"# delta_fw = {self.delta_fw_box.value()}\n")
                f.write(f"# R_ref = {self.r_ref_box.value()} kΩ\n")
                f.write(f"# C_ref = {self.c_ref_box.value()} pF\n")
                f.write(f"# Sample rate = {self.sample_rate_box.value()} kHz\n")
                f.write(f"# Start delay = {self.start_delay_box.value()} ms\n")
                f.write(f"# Sample delay = {self.sample_delay_box.value()} ms\n")
                f.write(f"# Average count = {self.average_count_box.value()}\n")
                f.write("\n")
                
                # Write header
                header = [
                    "freq_word", "freq_hz", "Z_mag_avg", "Z_phase_avg_deg",
                    "measurement_idx", "V_complex_real", "V_complex_imag",
                    "I_complex_real", "I_complex_imag", "A_v", "B_v", "A_i", "B_i",
                    "dc_ch0", "dc_ch1", "rms_residuals_ch0", "rms_residuals_ch1",
                    "voltage_phase_rad"
                ]
                f.write(",".join(header) + "\n")
                
                # Write data
                for freq_data in self.detailed_data:
                    freq_word = freq_data['freq_word']
                    freq_hz = freq_data['freq_hz']
                    Z_mag_avg = freq_data['Z_mag_avg']
                    Z_phase_avg = freq_data['Z_phase_avg']
                    
                    for measurement in freq_data['measurements']:
                        row = [
                            str(freq_word),
                            f"{freq_hz:.6f}",
                            f"{Z_mag_avg:.6f}",
                            f"{Z_phase_avg:.6f}",
                            str(measurement['measurement_idx']),
                            f"{measurement['V_complex_real']:.6f}",
                            f"{measurement['V_complex_imag']:.6f}",
                            f"{measurement['I_complex_real']:.6f}",
                            f"{measurement['I_complex_imag']:.6f}",
                            f"{measurement['A_v']:.6f}",
                            f"{measurement['B_v']:.6f}",
                            f"{measurement['A_i']:.6f}",
                            f"{measurement['B_i']:.6f}",
                            f"{measurement['dc_ch0']:.6f}",
                            f"{measurement['dc_ch1']:.6f}",
                            f"{measurement['rms_residuals_ch0']:.6f}",
                            f"{measurement['rms_residuals_ch1']:.6f}",
                            f"{measurement['voltage_phase_rad']:.6f}"
                        ]
                        f.write(",".join(row) + "\n")
            
            self.status.setText(f"Data saved to {filename}")
            
            # Generate new filename for next save
            self.generate_new_filename()
            
        except Exception as e:
            self.status.setText(f"Error saving file: {str(e)}")


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
