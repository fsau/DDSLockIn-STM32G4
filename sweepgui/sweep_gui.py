"""
Main GUI window for the Impedance Analyzer application.
Contains all UI components and business logic.
"""

import numpy as np
from PyQt5.QtWidgets import QMainWindow, QWidget, QHBoxLayout, QFormLayout
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
        
        # NEW: Track if we're starting a new sweep in hold mode
        self.new_sweep_in_hold = False
        
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
        
        self.fo_box = QSpinBox()
        self.fo_box.setRange(1, 1_000_000)
        self.fo_box.setValue(32800)
        
        self.delta_fw_box = QSpinBox()
        self.delta_fw_box.setRange(1, 1000)
        self.delta_fw_box.setValue(1)
        
        # Calibration controls
        self.r_ref_box = QDoubleSpinBox()
        self.r_ref_box.setRange(0.1, 100000)
        self.r_ref_box.setValue(2000.0)
        self.r_ref_box.setDecimals(1)
        
        self.c_ref_box = QDoubleSpinBox()
        self.c_ref_box.setRange(0.1, 10000)
        self.c_ref_box.setValue(10.5)
        self.c_ref_box.setDecimals(1)
        
        # Sampling controls
        self.sample_rate_box = QDoubleSpinBox()
        self.sample_rate_box.setRange(1000, 1000000)
        self.sample_rate_box.setValue(2833.3333)
        self.sample_rate_box.setDecimals(3)
        
        self.start_delay_box = QDoubleSpinBox()
        self.start_delay_box.setRange(0, 10000)
        self.start_delay_box.setValue(0.0)
        self.start_delay_box.setDecimals(1)
        
        self.sample_delay_box = QDoubleSpinBox()
        self.sample_delay_box.setRange(0, 10000)
        self.sample_delay_box.setValue(0.0)
        self.sample_delay_box.setDecimals(1)
        
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
        self.status.setMinimumHeight(110)
        self.status.setMaximumHeight(110)
        self.status.setText("Idle")
        self.status.setAlignment(Qt.AlignRight)
        
        # Add rows to form
        form.addRow("fi [Hz]:", self.fi_box)
        form.addRow("fo [Hz]:", self.fo_box)
        form.addRow("Δfw per step:", self.delta_fw_box)
        form.addRow("R_ref (kΩ):", self.r_ref_box)
        form.addRow("C_ref (pF):", self.c_ref_box)
        form.addRow("Sample Rate (kHz):", self.sample_rate_box)
        form.addRow("Start Delay (ms):", self.start_delay_box)
        form.addRow("Sample Delay (ms):", self.sample_delay_box)
        form.addRow("Average Count:", self.average_count_box)
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
        
    def start_sweep(self):
        """Start the frequency sweep"""
        fi = self.fi_box.value()
        fo = self.fo_box.value()
        
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
            self.new_sweep_in_hold = False  # Reset flag
        else:
            # When holding, clear only the detailed data (for CSV)
            # But keep the frequency/magnitude/phase arrays for plotting
            self.serial_sweep.detailed_data.clear()
            # Set flag to indicate new sweep in hold mode
            self.new_sweep_in_hold = True
            # Don't clear the plot
        
        # Set plot range
        self.plot_manager.set_xrange(fi, fo)
        
        # Start sweep with clear_data flag
        delay = self.serial_sweep.start_sweep(fi, fo, delta_fw, start_delay_ms, clear_data)
        
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
        # Reset the hold flag
        self.new_sweep_in_hold = False
    
    def on_sweep_step_callback(self, measurements, fw):
        """Callback for each sweep step"""
        if not measurements:
            return None
        
        # Calculate averages
        all_mag_measurements = []
        all_phase_measurements = []
        freq_detailed_data = []
        
        sample_rate = self.sample_rate_box.value() * 1000
        
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
            
            # Store for averaging
            if Z_mag > 0 and np.isfinite(Z_mag) and np.isfinite(Z_phase):
                all_mag_measurements.append(Z_mag)
                all_phase_measurements.append(Z_phase)
            
            # Store detailed data
            measurement_data = {
                'measurement_idx': measurement['measurement_idx'],
                'freq_word': measurement['freq_word'],
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
                'dc_ch0': C_v,
                'dc_ch1': C_i,
                'rms_residuals_ch0': rms_residuals_ch0,
                'rms_residuals_ch1': rms_residuals_ch1,
                'voltage_phase_rad': voltage_phase
            }
            freq_detailed_data.append(measurement_data)
            
            # Update status during averaging
            if self.average_count_box.value() > 1:
                status_text = f"f = {freq_hz:.1f} Hz\nAvg {measurement['measurement_idx']+1}/{self.average_count_box.value()}"
                self.status.setText(status_text)
        
        # Calculate averages
        if all_mag_measurements:
            avg_mag = np.mean(all_mag_measurements)
            avg_phase = np.mean(all_phase_measurements)
        else:
            avg_mag = 0
            avg_phase = 0
        
        # Update oscilloscope plot with last measurement
        if measurements:
            last_measurement = measurements[-1]
            ch0 = last_measurement['ch0']
            ch1 = last_measurement['ch1']
            freq_hz = last_measurement['freq_hz']
            
            # Get voltage phase from last calculation
            (_, _, voltage_phase, _, _, _, _, _, _, _, _, _, _) = \
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
            self.plot_manager.update_oscilloscope(t_aligned, ch0, ch1)
            
            # Set x-range for oscilloscope
            if freq_hz > 0:
                period_us = 1e6 / freq_hz
                self.plot_manager.plot.setXRange(0, 6 * period_us)
        
        # Update analyzer plot
        if self.hold_plot_checkbox.isChecked():
            # In hold mode, add point to existing plot
            self.plot_manager.add_measurement(
                freq_hz, avg_mag, avg_phase, 
                is_new_sweep=self.new_sweep_in_hold
            )
            # Reset the flag after first point
            if self.new_sweep_in_hold:
                self.new_sweep_in_hold = False
        else:
            # Not in hold mode, update entire plot
            self.plot_manager.update_analyzer(
                self.serial_sweep.frequencies + [freq_hz],
                self.serial_sweep.impedance_magnitudes + [avg_mag],
                self.serial_sweep.phases + [avg_phase]
            )
        
        # Update status
        freq_display = format_frequency(freq_hz)
        z_display = format_impedance(avg_mag)
        
        avg_count = self.average_count_box.value()
        if avg_count > 1:
            status_text = (
                f"f = {freq_display}\n"
                f"|Z| = {z_display} (avg of {len(all_mag_measurements)})\n"
                f"φ = {avg_phase:.1f}°\n"
                f"Residuals: V={rms_residuals_ch0*1e3:.2f}mV, I={rms_residuals_ch1*1e3:.2f}mV\n"
                f"DC: V={C_v*1e3:.1f}mV, I={C_i*1e3:.1f}mV"
            )
        else:
            status_text = (
                f"f = {freq_display}\n"
                f"|Z| = {z_display}\n"
                f"φ = {avg_phase:.1f}°\n"
                f"Residuals: V={rms_residuals_ch0*1e3:.2f}mV, I={rms_residuals_ch1*1e3:.2f}mV\n"
                f"DC: V={C_v*1e3:.1f}mV, I={C_i*1e3:.1f}mV"
            )
        
        self.status.setText(status_text)
        self.status.verticalScrollBar().setValue(self.status.verticalScrollBar().maximum())
        
        # Return data for storage
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
        """Execute one sweep step"""
        delta_fw = self.delta_fw_box.value()
        average_count = self.average_count_box.value()
        sample_delay_ms = self.sample_delay_box.value()
        
        continue_sweep, error = self.serial_sweep.sweep_step(
            delta_fw, average_count, sample_delay_ms
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