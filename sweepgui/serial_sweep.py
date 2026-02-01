import time
import serial
import serial.tools.list_ports
import numpy as np
from PyQt5.QtCore import QTimer

from utils import fs  # Import fs from utils

class SerialSweep:
    def __init__(self):
        # Find and open serial port
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            raise RuntimeError("No serial ports found")
        
        self.ser = serial.Serial(
            ports[-1].device,
            baudrate=115200,
            timeout=0.2
        )
        
        # State variables
        self.fw = 0
        self.fw_end = 0
        self.sweep_started = False
        self.waiting_for_start = False
        self.current_amplitude = 0
        
        # Data storage
        self.frequencies = []
        self.impedance_magnitudes = []
        self.phases = []
        self.detailed_data = []
        
        # Callbacks
        self.on_sweep_step = None
        self.on_sweep_complete = None
        
    def flush_input(self):
        """Flush serial input buffer"""
        self.ser.reset_input_buffer()
        time.sleep(0.05)
        while self.ser.in_waiting:
            self.ser.read(self.ser.in_waiting)
            time.sleep(0.01)
    
    def start_sweep(self, fi, fo, delta_fw, amplitude=0, start_delay_ms=0, clear_data=True):
        """Start a frequency sweep"""
        self.fw = fs(fi)  # Now fs is imported
        self.fw_end = fs(fo)
        self.current_amplitude = amplitude
        
        # Clear previous data only if requested
        if clear_data:
            self.frequencies.clear()
            self.impedance_magnitudes.clear()
            self.phases.clear()
            self.detailed_data.clear()
        
        self.sweep_started = False
        self.waiting_for_start = True
        
        # Set initial frequency
        cmd = f"ff{self.fw}a\r\n".encode()
        self.ser.write(cmd)
        
        # Set amplitude if non-zero
        if amplitude > 0:
            time.sleep(0.001)  # Small delay
            amp_cmd = f"aa{amplitude:03d}a\r\n".encode()
            self.ser.write(amp_cmd)
        
        return start_delay_ms
    
    def begin_sweep(self):
        """Begin the actual sweep after delay"""
        self.waiting_for_start = False
        self.sweep_started = True
        return True
    
    def stop_sweep(self):
        """Stop the sweep"""
        self.sweep_started = False
        self.waiting_for_start = False
    
    def perform_measurement(self, fw, amplitude=0, average_count=1, sample_delay_ms=0):
        """Perform measurement at a specific frequency word"""
        all_mag_measurements = []
        all_phase_measurements = []
        freq_detailed_data = []
        
        for measurement_idx in range(average_count):
            # Send frequency
            cmd = f"ff{fw}aa{amplitude:03d}xx".encode()
            self.ser.write(cmd)
            time.sleep(0.001)
            
            self.ser.reset_output_buffer()

            # Apply sample delay
            if sample_delay_ms > 0:
                time.sleep(sample_delay_ms / 1000.0)

            # Trigger measurement
            self.ser.write(b"m")
            raw = self.ser.read(1024 * 4)

            if len(raw) != 1024 * 4:
                return None, "Short read"

            # Convert data
            raw_u8 = np.frombuffer(raw, dtype=np.uint8)
            data = raw_u8[0::2].astype(np.int16) + \
                   (raw_u8[1::2].astype(np.int16) << 8)

            if data.size != 2048:
                return None, "Frame error"

            half = data.size // 2
            ch0 = (data[:half]-2048)*3.3/4096  # V_dut
            ch1 = (data[half:]-2048)*3.3/4096  # I_ref
            
            freq_hz = fw / (2**28) * 25e6
            
            # Store measurement data
            measurement_data = {
                'measurement_idx': measurement_idx,
                'freq_word': fw,
                'freq_hz': freq_hz,
                'amplitude': amplitude,
                'ch0': ch0,
                'ch1': ch1
            }
            freq_detailed_data.append(measurement_data)
        
        return freq_detailed_data, None
    
    def sweep_step(self, delta_fw, amplitude=0, average_count=1, sample_delay_ms=0):
        """Execute one step of the sweep"""
        if self.waiting_for_start or not self.sweep_started:
            return True, None
        
        if self.fw > self.fw_end:
            self.sweep_started = False
            if self.on_sweep_complete:
                self.on_sweep_complete()
            return False, "Done"
        
        # Check if amplitude changed
        if amplitude != self.current_amplitude:
            # Update amplitude
            if amplitude > 0:
                amp_cmd = f"aa{amplitude:03d}a\r\n".encode()
                self.ser.write(amp_cmd)
                time.sleep(0.001)
            self.current_amplitude = amplitude
        
        # Perform measurement
        measurements, error = self.perform_measurement(
            self.fw, amplitude, average_count, sample_delay_ms
        )
        
        if error:
            self.sweep_started = False
            return False, error
        
        # Call sweep step callback
        if self.on_sweep_step:
            result = self.on_sweep_step(measurements, self.fw)
            if result:
                # Data arrays (frequencies, magnitudes, phases) are already updated in callback
                # Only store detailed data for CSV
                self.detailed_data.append(result['detailed_data'])
        
        # Increment frequency
        self.fw += delta_fw
        return True, None