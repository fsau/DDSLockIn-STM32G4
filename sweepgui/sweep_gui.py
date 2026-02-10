"""
Main GUI window for the Impedance Analyzer application.
Contains all UI components and business logic.
"""

import numpy as np
from PyQt5.QtWidgets import QMainWindow, QWidget, QHBoxLayout, QFormLayout, QApplication
from PyQt5.QtWidgets import QSpinBox, QDoubleSpinBox, QPushButton, QTextEdit, QLineEdit, QLabel, QCheckBox
from PyQt5.QtCore import QTimer, Qt

from serial_sweep import SerialSweep
from impedance_calculator import ImpedanceCalculator
from csv_saver import CSVSaver
from plot_manager import PlotManager
from utils import format_frequency, format_impedance, generate_new_filename

class SweepGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        
        self.setWindowTitle("Serial Sweep")
        self.resize(1000, 500)
        
        # Initialize components
        self.serial_sweep = SerialSweep()
        self.plot_manager = PlotManager()
        self.impedance_calculator = None
        self.csv_saver = None
        
        self.insert_nan_next_point = False
        
        # NEW: Amplitude control state
        self.last_amplitude = 100  # Store last amplitude used
        
        # Setup UI
        self.setup_ui()
        
        # Initialize state
        self.timer = QTimer()
        self.timer.timeout.connect(self.sweep_step)
        
        # Set callbacks
        self.serial_sweep.on_sweep_step = self.on_sweep_step_callback
        self.serial_sweep.on_sweep_complete = self.on_sweep_complete_callback
        
        # Generate initial filename
        self.generate_new_filename()
        
    def setup_ui(self):
        """Setup the user interface"""
        central = QWidget()
        self.setCentralWidget(central)
        layout = QHBoxLayout(central)
        
        # Add plot widget
        layout.addWidget(self.plot_manager.plot_widget, stretch=3)
        
        # Create controls panel
        controls = self.create_controls()
        layout.addWidget(controls, stretch=1)
    
    def create_controls(self):
        """Create the controls panel"""
        controls = QWidget()
        form = QFormLayout(controls)
        
        meas_label = QLabel("--- Measurement Options ---")
        meas_label.setAlignment(Qt.AlignCenter)
        meas_label.setStyleSheet("color: #888; font-weight: bold;")
        
        form.addRow(meas_label)
        
        # Frequency controls
        self.fi_box = QSpinBox()
        self.fi_box.setRange(1, 1_000_000)
        self.fi_box.setValue(32720)
        self.fi_box.setSuffix(" Hz")
        
        self.fo_box = QSpinBox()
        self.fo_box.setRange(1, 1_000_000)
        self.fo_box.setValue(32800)
        self.fo_box.setSuffix(" Hz")
        
        self.delta_fw_box = QSpinBox()
        self.delta_fw_box.setRange(1, 1000)
        self.delta_fw_box.setValue(1)
        
        # Amplitude control
        self.amplitude_box = QSpinBox()
        self.amplitude_box.setRange(0, 100)
        self.amplitude_box.setValue(100)
        self.amplitude_box.setSuffix(" %")
        
        # NEW: Automatic amplitude control checkbox
        self.auto_amplitude_checkbox = QCheckBox()
        self.auto_amplitude_checkbox.setChecked(False)
        
        # # NEW: Auto delay checkbox
        # self.auto_delay_checkbox = QCheckBox()
        # self.auto_delay_checkbox.setChecked(False)
        
        # Calibration controls
        self.r_ref_box = QDoubleSpinBox()
        self.r_ref_box.setRange(0.1, 100000)
        self.r_ref_box.setValue(2000.0)
        self.r_ref_box.setDecimals(1)
        self.r_ref_box.setSuffix(" kΩ")
        
        self.c_ref_box = QDoubleSpinBox()
        self.c_ref_box.setRange(0.1, 10000)
        self.c_ref_box.setValue(10.5)
        self.c_ref_box.setDecimals(1)
        self.c_ref_box.setSuffix(" pF")
        
        # Sampling controls
        self.sample_rate_box = QDoubleSpinBox()
        self.sample_rate_box.setRange(1, 10000)
        self.sample_rate_box.setValue(1000.0)
        self.sample_rate_box.setDecimals(3)
        self.sample_rate_box.setSuffix(" kHz")
        
        self.start_delay_box = QDoubleSpinBox()
        self.start_delay_box.setRange(0, 10000)
        self.start_delay_box.setValue(0.0)
        self.start_delay_box.setDecimals(1)
        self.start_delay_box.setSuffix("  ms")
        
        self.sample_delay_box = QDoubleSpinBox()
        self.sample_delay_box.setRange(0, 10000)
        self.sample_delay_box.setValue(0.0)
        self.sample_delay_box.setDecimals(1)
        self.sample_delay_box.setSuffix("  ms")
        
        self.average_count_box = QSpinBox()
        self.average_count_box.setRange(1, 100)
        self.average_count_box.setValue(1)
        
        # Hold plot checkbox
        self.hold_plot_checkbox = QCheckBox()
        self.hold_plot_checkbox.setChecked(False)
        
        # Buttons
        self.start_btn = QPushButton("Start / Update")
        self.stop_btn = QPushButton("Stop")
        self.clear_btn = QPushButton("Clear Data")
        self.save_btn = QPushButton("Save to CSV")
        
        # Filename and notes
        self.filename_edit = QLineEdit()
        self.filename_edit.setText("sweep_data_001.csv")
        
        self.notes_edit = QTextEdit()
        self.notes_edit.setMinimumHeight(60)
        self.notes_edit.setPlaceholderText("Enter notes for this measurement...")
        
        # Status display
        self.status = QTextEdit()
        self.status.setReadOnly(True)
        self.status.setMinimumHeight(120)
        self.status.setMaximumHeight(120)
        self.status.setText("Idle")
        self.status.setAlignment(Qt.AlignRight)
        
        # Add rows to form
        form.addRow("f initial:", self.fi_box)
        form.addRow("f final:", self.fo_box)
        form.addRow("Δfw per step:", self.delta_fw_box)
        form.addRow("Amplitude:", self.amplitude_box)
        form.addRow("Auto Amp:", self.auto_amplitude_checkbox)  # NEW: Auto amplitude checkbox
        # form.addRow("Auto Delay:", self.auto_delay_checkbox)    # NEW: Auto delay checkbox
        form.addRow("R_ref:", self.r_ref_box)
        form.addRow("C_ref:", self.c_ref_box)
        form.addRow("Sample Rate:", self.sample_rate_box)
        form.addRow("Start Delay", self.start_delay_box)
        form.addRow("Sample Delay", self.sample_delay_box)
        form.addRow("Samples Average:", self.average_count_box)
        form.addRow("Hold Plot:", self.hold_plot_checkbox)
        form.addRow(self.start_btn)
        form.addRow(self.stop_btn)
        form.addRow(self.clear_btn)
        
        # Add separator
        separator_widget = QWidget()
        separator_layout = QHBoxLayout(separator_widget)
        separator_layout.setContentsMargins(0, 10, 0, 10)  # Add some vertical spacing
        
        save_label = QLabel("--- Save Options ---")
        save_label.setAlignment(Qt.AlignCenter)
        save_label.setStyleSheet("color: #888; font-weight: bold;")
        
        separator_layout.addWidget(save_label)
        form.addRow(separator_widget)
        
        form.addRow("Filename:", self.filename_edit)
        form.addRow("Notes:", self.notes_edit)
        form.addRow(self.save_btn)
        form.addRow(self.status)
        
        # Connect signals
        self.start_btn.clicked.connect(self.start_sweep)
        self.stop_btn.clicked.connect(self.stop_sweep)
        self.clear_btn.clicked.connect(self.clear_data)
        self.save_btn.clicked.connect(self.save_to_csv)
        
        return controls
    
    def generate_new_filename(self):
        """Generate new filename with auto-numbering"""
        current_filename = self.filename_edit.text().strip()
        new_filename = generate_new_filename(current_filename)
        self.filename_edit.blockSignals(True)
        self.filename_edit.setText(new_filename)
        self.filename_edit.blockSignals(False)
    
    def calculate_optimal_amplitude(self, amplitude, max_amplitude, A_v, B_v, A_i, B_i):
        """
        Calculate optimal amplitude based on fitted amplitudes.
        Returns new amplitude value (0-100) and True if amplitude was adjusted.
        """
        # Calculate fitted amplitudes for both channels
        amp_v = np.sqrt(A_v**2 + B_v**2)
        amp_i = np.sqrt(A_i**2 + B_i**2)
        
        # Target range for optimal measurement (volts)
        MIN_TARGET_AMPLITUDE = 0.79  # V - minimum for good SNR
        MAX_ALLOWED_AMPLITUDE = 0.8  # V - maximum to avoid clipping (with margin)
        
        # Get the larger of the two amplitudes (worst case for clipping)
        max_measured_amp = max(amp_v, amp_i)
        
        if max_measured_amp > 0:
            # Check if amplitude is TOO HIGH (risk of clipping)
            if max_measured_amp > MAX_ALLOWED_AMPLITUDE:
                # Reduce amplitude proportionally to bring it to MAX_ALLOWED_AMPLITUDE
                scale_factor = MAX_ALLOWED_AMPLITUDE / max_measured_amp
                new_amp = max(1, int(amplitude * scale_factor))
                return new_amp, True
            
            # Check if amplitude is TOO LOW (poor SNR)
            elif max_measured_amp < MIN_TARGET_AMPLITUDE and amplitude < max_amplitude:
                # Increase amplitude proportionally to bring it to MIN_TARGET_AMPLITUDE
                scale_factor = MIN_TARGET_AMPLITUDE / max_measured_amp
                # Increase amplitude, but not above max_amplitude
                new_amp = min(max_amplitude, int(amplitude * scale_factor))
                # Cap increase to avoid overshoot (max 200% increase per step)
                if new_amp > amplitude * 2.0:
                    new_amp = int(amplitude * 2.0)
                return new_amp, True
        
        return amplitude, False
    
    def check_stability(self, measurements, current_amplitude=None):
        """Check if measurements are stable with amplitude-based adaptive threshold"""
        if not measurements or len(measurements) < 3:
            return False
        
        sample_rate = self.sample_rate_box.value() * 1000
        impedance_readings = []
        
        for measurement in measurements:
            ch0 = measurement['ch0']
            ch1 = measurement['ch1']
            freq_hz = measurement['freq_hz']
            
            # Calculate impedance
            (Z_mag, Z_phase, _, _, _, _, _, _, _, _, _, _, _, _) = \
                self.impedance_calculator.calculate_impedance_lsq(
                    ch0, ch1, freq_hz, sample_rate
                )
            
            if Z_mag > 0 and np.isfinite(Z_mag):
                impedance_readings.append(Z_mag)
        
        if len(impedance_readings) < 3:
            return False
        
        # Get last 3 measurements
        last_three = impedance_readings[-3:]
        
        # Calculate variation
        max_val = max(last_three)
        min_val = min(last_three)
        
        if max_val <= 0:
            return False
        
        variation = (max_val - min_val) / max_val
        
        # Adaptive threshold based on amplitude
        # When amplitude is low, signal is weak → be more tolerant (higher threshold)
        # When amplitude is high, signal is strong → be stricter (lower threshold)
        if current_amplitude is None:
            current_amplitude = 100  # Default to max if not provided
        
        if current_amplitude < 10:  # < 10% amplitude - very weak signal
            threshold = 0.20  # 20% variation allowed
        elif current_amplitude < 20:  # < 20% amplitude - weak signal
            threshold = 0.15  # 15% variation allowed
        elif current_amplitude < 50:  # < 50% amplitude - moderate signal
            threshold = 0.10  # 10% variation allowed
        elif current_amplitude < 80:  # < 80% amplitude - good signal
            threshold = 0.07  # 7% variation allowed
        else:  # 80-100% amplitude - excellent signal
            threshold = 0.05  # 5% variation allowed
        
        # Also consider the impedance magnitude itself
        # For very low impedances, be even more tolerant
        avg_impedance = np.mean(last_three)
        if avg_impedance < 1:  # < 1Ω
            threshold = max(threshold, 0.25)  # At least 25% tolerance
        elif avg_impedance < 10:  # < 10Ω
            threshold = max(threshold, 0.20)  # At least 20% tolerance
        elif avg_impedance < 100:  # < 100Ω
            threshold = max(threshold, 0.15)  # At least 15% tolerance
        
        return variation < threshold
    
    # def perform_measurement_with_auto_delay(self, fw, amplitude, average_count):
    #     """Perform measurement with automatic delay until stable"""
    #     max_retries = 10
    #     base_delay = self.sample_delay_box.value()
    #     min_measurements_for_check = max(3, average_count)
        
    #     all_measurements = []
        
    #     for retry in range(max_retries):
    #         current_delay = base_delay + (retry * 100)  # Increase delay by 100ms each retry
            
    #         # Perform measurements
    #         measurements, error = self.serial_sweep.perform_measurement(
    #             fw, amplitude, min_measurements_for_check, current_delay
    #         )
            
    #         if error:
    #             return measurements, error
            
    #         # Add to all measurements
    #         all_measurements.extend(measurements)
            
    #         # Check if we have enough measurements to assess stability
    #         if len(all_measurements) >= 3:
    #             # Check stability with amplitude-aware adaptive threshold
    #             if self.check_stability(all_measurements, amplitude):
    #                 # Stable! Return only the requested number of measurements
    #                 # Use the most recent measurements
    #                 if len(all_measurements) > average_count:
    #                     return all_measurements[-average_count:], None
    #                 else:
    #                     return all_measurements, None
            
    #         # Update status without freezing
    #         if retry > 0:
    #             self.status.setText(f"Auto delay: retry {retry}, delay: {current_delay}ms")
    #             # Process events to update UI but don't sleep
    #             QApplication.processEvents()
        
    #     # If we get here, we've used all retries
    #     # Return the most recent measurements anyway
    #     if len(all_measurements) > average_count:
    #         return all_measurements[-average_count:], None
    #     else:
    #         return all_measurements, None
    
    def start_sweep(self):
        """Start the frequency sweep"""
        fi = self.fi_box.value()
        fo = self.fo_box.value()
        amplitude = self.amplitude_box.value()
        
        if fo < fi:
            self.status.setText("fo < fi")
            return
        
        # Initialize impedance calculator with current values
        self.impedance_calculator = ImpedanceCalculator(
            self.r_ref_box.value(),
            self.c_ref_box.value()
        )
        
        # Get sweep parameters
        delta_fw = self.delta_fw_box.value()
        start_delay_ms = self.start_delay_box.value()
        
        # Clear previous data if not holding
        clear_data = not self.hold_plot_checkbox.isChecked()
        
        if clear_data:
            # Clear everything including plots
            self.serial_sweep.frequencies.clear()
            self.serial_sweep.impedance_magnitudes.clear()
            self.serial_sweep.phases.clear()
            self.serial_sweep.detailed_data.clear()
            self.plot_manager.clear_plots()
            self.insert_nan_next_point = False  # Reset flag
        else:
            # When holding, clear only the detailed data (for CSV)
            # But keep the frequency/magnitude/phase arrays for plotting
            self.serial_sweep.detailed_data.clear()
            # Set flag to insert nan before first point of new sweep
            self.insert_nan_next_point = True
            # Don't clear the plot
        
        # Reset amplitude control state
        self.last_amplitude = amplitude
        
        # Set plot range
        self.plot_manager.set_xrange(fi, fo)
        
        # Start sweep with clear_data flag
        delay = self.serial_sweep.start_sweep(fi, fo, delta_fw, amplitude, start_delay_ms, clear_data)
        
        if delay > 0:
            self.status.setText(f"Waiting {delay}ms before starting...")
            QTimer.singleShot(int(delay), self.begin_sweep)
        else:
            self.begin_sweep()
    
    def begin_sweep(self):
        """Begin the sweep after delay"""
        if self.serial_sweep.begin_sweep():
            self.status.setText("Sweeping...")
            self.timer.start(1)
    
    def stop_sweep(self):
        """Stop the sweep"""
        self.serial_sweep.stop_sweep()
        self.timer.stop()
        self.status.setText("Stopped")
    
    def clear_data(self):
        """Clear all data"""
        # Always clear when explicitly requested
        self.serial_sweep.frequencies.clear()
        self.serial_sweep.impedance_magnitudes.clear()
        self.serial_sweep.phases.clear()
        self.serial_sweep.detailed_data.clear()
        self.plot_manager.clear_plots()
        self.status.setText("Data cleared")
        self.generate_new_filename()
        # Also uncheck hold plot when clearing
        self.hold_plot_checkbox.setChecked(False)
        # Reset the nan flag
        self.insert_nan_next_point = False
    
    def on_sweep_step_callback(self, measurements, fw):
        """Callback for each sweep step"""
        if not measurements:
            return None
        
        # Calculate averages
        all_mag_measurements = []
        all_phase_measurements = []
        freq_detailed_data = []
        
        sample_rate = self.sample_rate_box.value() * 1000
        
        # Store the last fitted parameters for amplitude control
        last_A_v, last_B_v, last_A_i, last_B_i = 0, 0, 0, 0
        
        for measurement in measurements:
            ch0 = measurement['ch0']
            ch1 = measurement['ch1']
            freq_hz = measurement['freq_hz']
            
            # Calculate impedance
            (Z_mag, Z_phase, voltage_phase, rms_residuals_ch0, rms_residuals_ch1,
            C_v, C_i, A_v, B_v, A_i, B_i, V_complex, I_complex) = \
                self.impedance_calculator.calculate_impedance_lsq(
                    ch0, ch1, freq_hz, sample_rate
                )
            
            # Store fitted parameters from last measurement
            last_A_v, last_B_v, last_A_i, last_B_i = A_v, B_v, A_i, B_i
            
            # Store for averaging
            if Z_mag > 0 and np.isfinite(Z_mag) and np.isfinite(Z_phase):
                all_mag_measurements.append(Z_mag)
                all_phase_measurements.append(Z_phase)
            
            # Store detailed data
            measurement_data = {
                'measurement_idx': measurement['measurement_idx'],
                'freq_word': measurement['freq_word'],
                'freq_hz': freq_hz,
                'amplitude': measurement['amplitude'],
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
                'dc_ch0': C_v,
                'dc_ch1': C_i,
                'rms_residuals_ch0': rms_residuals_ch0,
                'rms_residuals_ch1': rms_residuals_ch1,
                'voltage_phase_rad': voltage_phase
            }
            freq_detailed_data.append(measurement_data)
        
        # Calculate averages
        if all_mag_measurements:
            avg_mag = np.mean(all_mag_measurements)
            avg_phase = np.mean(all_phase_measurements)
        else:
            avg_mag = 0
            avg_phase = 0
        
        # NEW: Automatic amplitude control
        current_amplitude = measurements[-1]['amplitude'] if measurements else self.amplitude_box.value()
        max_amplitude = self.amplitude_box.value()  # User's maximum allowed amplitude
        
        if self.auto_amplitude_checkbox.isChecked() and measurements:
            # Calculate optimal amplitude based on last measurement
            optimal_amp, amp_adjusted = self.calculate_optimal_amplitude(
                current_amplitude, max_amplitude,
                last_A_v, last_B_v, last_A_i, last_B_i
            )
            
            if amp_adjusted and optimal_amp != current_amplitude:
                # Store new amplitude for next step
                self.last_amplitude = optimal_amp
                # Update the current amplitude in serial_sweep for next step
                self.serial_sweep.current_amplitude = optimal_amp
        
        # Update oscilloscope plot with last measurement
        if measurements:
            last_measurement = measurements[-1]
            ch0 = last_measurement['ch0']
            ch1 = last_measurement['ch1']
            freq_hz = last_measurement['freq_hz']
            
            # Get voltage phase from last calculation
            (_, _, voltage_phase, _, _, C_v, C_i, A_v, B_v, A_i, B_i, _, _) = \
                self.impedance_calculator.calculate_impedance_lsq(
                    ch0, ch1, freq_hz, sample_rate
                )
            
            # Create time-aligned plot
            n_samples = len(ch0)
            t = np.arange(n_samples) / sample_rate * 1e6
            
            if freq_hz > 0:
                t_offset = voltage_phase / (2 * np.pi * freq_hz) * 1e6
            else:
                t_offset = 0
            
            t_aligned = t + t_offset - 2e6/(freq_hz)
            
            # Pass fit parameters to plot manager
            fit_params = {
                'A_v': A_v, 'B_v': B_v, 'C_v': C_v,
                'A_i': A_i, 'B_i': B_i, 'C_i': C_i
            }
            self.plot_manager.update_oscilloscope(t_aligned, ch0, ch1, fit_params)
            
            # Set x-range for oscilloscope
            if freq_hz > 0:
                period_us = 1e6 / freq_hz
                self.plot_manager.plot.setXRange(0, 6 * period_us)
        
        # Check if we need to insert nan BEFORE adding the new point
        if self.hold_plot_checkbox.isChecked() and self.insert_nan_next_point:
            # Insert nan values to break the line between sweeps
            self.serial_sweep.frequencies.append(np.nan)
            self.serial_sweep.impedance_magnitudes.append(np.nan)
            self.serial_sweep.phases.append(np.nan)
            self.insert_nan_next_point = False
        
        # Append the current measurement data to the serial_sweep arrays
        # This ensures the plot has all data including the current point
        self.serial_sweep.frequencies.append(freq_hz)
        self.serial_sweep.impedance_magnitudes.append(avg_mag)
        self.serial_sweep.phases.append(avg_phase)
        
        # Update analyzer plot with ALL accumulated data
        self.plot_manager.update_analyzer(
            self.serial_sweep.frequencies,
            self.serial_sweep.impedance_magnitudes,
            self.serial_sweep.phases
        )
        
        # Update status
        freq_display = format_frequency(freq_hz)
        z_display = format_impedance(avg_mag)
        
        avg_count = self.average_count_box.value()
        
        # Calculate signal amplitudes for status display
        amp_v = np.sqrt(last_A_v**2 + last_B_v**2)
        amp_i = np.sqrt(last_A_i**2 + last_B_i**2)
        
        # Include amplitude info in status if auto amplitude is enabled
        amp_info = ""
        if self.auto_amplitude_checkbox.isChecked():
            amp_info = f"Amp: {current_amplitude}% (V={amp_v*1000:.0f}mV, I={amp_i*1000:.0f}mV)\n"
        
        if avg_count > 1:
            status_text = (
                f"f = {freq_display}\n"
                f"{amp_info}"
                f"|Z| = {z_display} (avg of {len(all_mag_measurements)})\n"
                f"φ = {avg_phase:.1f}°\n"
                f"Residuals: V={rms_residuals_ch0*1e3:.2f}mV, I={rms_residuals_ch1*1e3:.2f}mV\n"
                f"DC: V={C_v*1e3:.1f}mV, I={C_i*1e3:.1f}mV"
            )
        else:
            status_text = (
                f"f = {freq_display}\n"
                f"{amp_info}"
                f"|Z| = {z_display}\n"
                f"φ = {avg_phase:.1f}°\n"
                f"Residuals: V={rms_residuals_ch0*1e3:.2f}mV, I={rms_residuals_ch1*1e3:.2f}mV\n"
                f"DC: V={C_v*1e3:.1f}mV, I={C_i*1e3:.1f}mV"
            )
        
        self.status.setText(status_text)
        self.status.verticalScrollBar().setValue(self.status.verticalScrollBar().maximum())
        
        # Return data for storage (serial_sweep will append this again, but that's OK)
        return {
            'freq_hz': freq_hz,
            'Z_mag_avg': avg_mag,
            'Z_phase_avg': avg_phase,
            'detailed_data': {
                'freq_word': fw,
                'freq_hz': freq_hz,
                'Z_mag_avg': avg_mag,
                'Z_phase_avg': avg_phase,
                'measurements': freq_detailed_data,
                'avg_count': self.average_count_box.value()
            }
        }
    
    def on_sweep_complete_callback(self):
        """Callback when sweep is complete"""
        self.timer.stop()
        self.status.setText("Done")
        self.generate_new_filename()
    
    def sweep_step(self):
        """Execute one sweep step - read ALL parameters in real-time"""
        # Read ALL parameters in real-time
        delta_fw = self.delta_fw_box.value()
        amplitude = self.amplitude_box.value()
        average_count = self.average_count_box.value()
        sample_delay_ms = self.sample_delay_box.value()
        
        # Update impedance calculator with current values
        self.impedance_calculator = ImpedanceCalculator(
            self.r_ref_box.value(),
            self.c_ref_box.value()
        )
        
        # NEW: Use auto-adjusted amplitude if enabled
        if self.auto_amplitude_checkbox.isChecked():
            amplitude = self.last_amplitude
        
        # # NEW: If auto delay is enabled, we need to override the normal sweep step
        # if self.auto_delay_checkbox.isChecked():
        #     if not self.serial_sweep.sweep_started or self.serial_sweep.waiting_for_start:
        #         return
            
        #     if self.serial_sweep.fw > self.serial_sweep.fw_end:
        #         self.serial_sweep.sweep_started = False
        #         if self.serial_sweep.on_sweep_complete:
        #             self.serial_sweep.on_sweep_complete()
        #         return
            
        #     # Perform measurement with auto delay
        #     measurements, error = self.perform_measurement_with_auto_delay(
        #         self.serial_sweep.fw, amplitude, average_count
        #     )
            
        #     if error:
        #         self.serial_sweep.sweep_started = False
        #         self.timer.stop()
        #         self.status.setText(error)
        #         return
            
        #     # Call sweep step callback
        #     if self.serial_sweep.on_sweep_step:
        #         result = self.serial_sweep.on_sweep_step(measurements, self.serial_sweep.fw)
        #         if result:
        #             # Only store detailed data for CSV
        #             self.serial_sweep.detailed_data.append(result['detailed_data'])
            
        #     # Increment frequency
        #     self.serial_sweep.fw += delta_fw
        # else:

        # Original sweep step logic
        continue_sweep, error = self.serial_sweep.sweep_step(
            delta_fw, amplitude, average_count, sample_delay_ms
        )
        
        if error:
            self.timer.stop()
            self.status.setText(error)
    
    def save_to_csv(self):
        """Save data to CSV file"""
        if not self.serial_sweep.detailed_data:
            self.status.setText("No data to save")
            return
        
        # Get parameters for CSV header
        parameters = {
            'fi': self.fi_box.value(),
            'fo': self.fo_box.value(),
            'delta_fw': self.delta_fw_box.value(),
            'Amplitude': f"{self.amplitude_box.value()}%",
            'Auto_amplitude': 'Yes' if self.auto_amplitude_checkbox.isChecked() else 'No',
            # 'Auto_delay': 'Yes' if self.auto_delay_checkbox.isChecked() else 'No',
            'R_ref': self.r_ref_box.value(),
            'C_ref': self.c_ref_box.value(),
            'Sample rate': self.sample_rate_box.value(),
            'Start delay': self.start_delay_box.value(),
            'Sample delay': self.sample_delay_box.value(),
            'Average count': self.average_count_box.value(),
            'Hold plot': 'Yes' if self.hold_plot_checkbox.isChecked() else 'No'
        }
        
        # Initialize CSV saver
        self.csv_saver = CSVSaver(parameters)
        
        # Get filename and notes
        filename = self.filename_edit.text().strip()
        if not filename:
            filename = "sweep_data_001.csv"
        
        notes = self.notes_edit.toPlainText()
        
        # Save data
        message, success = self.csv_saver.save_to_csv(
            filename, notes, self.serial_sweep.detailed_data
        )
        
        self.status.setText(message)
        if success:
            self.generate_new_filename()