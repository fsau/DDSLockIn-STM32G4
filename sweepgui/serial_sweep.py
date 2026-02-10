import time
import serial
import serial.tools.list_ports
import numpy as np
from PyQt5.QtCore import QTimer, QMutex, pyqtSignal, QObject
import threading
from collections import deque
import queue

from utils import fs

class SerialSweep(QObject):
    # Signals for async communication
    data_ready = pyqtSignal(object)
    error_signal = pyqtSignal(str)
    connection_status = pyqtSignal(bool)  # True=connected, False=disconnected
    
    def __init__(self):
        super().__init__()
        
        # Serial connection state
        self.ser = None
        self.port_name = None
        self.is_connected = False
        self.connection_mutex = threading.RLock()
        
        # Communication settings
        self.baudrate = 115200
        self.read_timeout = 1.0  # 1 second timeout for reads
        self.write_timeout = 0.5  # 0.5 second timeout for writes
        self.command_retry_count = 2
        
        # FIFO buffer for receiving data in background
        self.receive_fifo = queue.Queue(maxsize=10000)  # 10KB buffer
        self.receive_thread = None
        self.receive_thread_running = False
        
        # State variables (compatible with original)
        self.fw = 0
        self.fw_end = 0
        self.sweep_started = False
        self.waiting_for_start = False
        self.current_amplitude = 0
        
        # Data storage (compatible with original)
        self.frequencies = []
        self.impedance_magnitudes = []
        self.phases = []
        self.detailed_data = []
        
        # Callbacks (compatible with original)
        self.on_sweep_step = None
        self.on_sweep_complete = None
        
        # Find and open port
        self._find_and_open_port()
        
        # Connection monitoring timer
        self.connection_timer = QTimer()
        self.connection_timer.timeout.connect(self._check_connection)
        self.connection_timer.start(2000)  # Check every 2 seconds
        
        # Command queue for async operations
        self.command_queue = deque()
        self.command_thread = None
        self.command_thread_running = False
        
        # Start background threads
        self._start_receive_thread()
        self._start_command_thread()
    
    def _start_receive_thread(self):
        """Start background thread for receiving data into FIFO"""
        self.receive_thread_running = True
        self.receive_thread = threading.Thread(
            target=self._receive_thread_func,
            daemon=True,
            name="SerialReceiveThread"
        )
        self.receive_thread.start()
    
    def _receive_thread_func(self):
        """Background thread that continuously reads from serial port into FIFO"""
        while self.receive_thread_running:
            try:
                # Only read if we have an open connection
                with self.connection_mutex:
                    if not self.is_connected or not self.ser or not self.ser.is_open:
                        time.sleep(0.1)
                        continue
                
                # Read any available data
                bytes_to_read = self.ser.in_waiting
                if bytes_to_read > 0:
                    try:
                        data = self.ser.read(bytes_to_read)
                        if data:
                            # Add to FIFO
                            try:
                                self.receive_fifo.put_nowait(data)
                            except queue.Full:
                                # FIFO full, drop oldest data
                                try:
                                    self.receive_fifo.get_nowait()
                                    self.receive_fifo.put_nowait(data)
                                except queue.Empty:
                                    pass
                    except serial.SerialException:
                        with self.connection_mutex:
                            self.is_connected = False
                            self.connection_status.emit(False)
                        time.sleep(0.5)
                else:
                    time.sleep(0.001)
                    
            except Exception:
                time.sleep(0.1)
    
    def _clear_fifo(self):
        """Clear the receive FIFO buffer"""
        try:
            while not self.receive_fifo.empty():
                self.receive_fifo.get_nowait()
        except queue.Empty:
            pass
    
    def _read_from_fifo(self, size, timeout=1.0):
        """
        Read data from FIFO with timeout.
        Returns: (success, data, error_message)
        """
        data = bytearray()
        start_time = time.time()
        
        while len(data) < size:
            if time.time() - start_time > timeout:
                return False, None, f"FIFO read timeout after {timeout} seconds"
            
            try:
                chunk = self.receive_fifo.get_nowait()
                if chunk:
                    data.extend(chunk)
            except queue.Empty:
                time.sleep(0.001)
        
        return True, bytes(data), None
    
    def _find_and_open_port(self):
        """Find and open serial port with error handling"""
        with self.connection_mutex:
            try:
                # Close existing connection
                if self.ser and hasattr(self.ser, 'is_open') and self.ser.is_open:
                    try:
                        self.ser.close()
                    except:
                        pass
                
                # Find available port
                ports = list(serial.tools.list_ports.comports())
                if not ports:
                    self.port_name = None
                    self.is_connected = False
                    self.connection_status.emit(False)
                    return False
                
                self.port_name = ports[-1].device
                
                # Try to open port
                self.ser = serial.Serial(
                    port=self.port_name,
                    baudrate=self.baudrate,
                    timeout=self.read_timeout,
                    write_timeout=self.write_timeout,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE
                )
                
                # Clear FIFO on new connection
                self._clear_fifo()
                
                # Test connection
                self._test_connection()
                
                self.is_connected = True
                self.connection_status.emit(True)
                return True
                
            except Exception:
                self.is_connected = False
                self.connection_status.emit(False)
                return False
    
    def _test_connection(self):
        """Test if connection is working"""
        if not self.ser or not self.ser.is_open:
            return False
            
        try:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            
            self.ser.write(b'\r\n')
            self.ser.flush()
            
            time.sleep(0.05)
            
            self._clear_fifo()
            
            return True
            
        except Exception:
            return False
    
    def _check_connection(self):
        """Periodically check connection status"""
        if not self.sweep_started:
            with self.connection_mutex:
                try:
                    if not self.ser or not self.ser.is_open:
                        self.is_connected = False
                        self.connection_status.emit(False)
                        return
                    
                    # Check if port is still accessible
                    dummy = self.ser.in_waiting
                    
                    if not self.is_connected:
                        self.is_connected = True
                        self.connection_status.emit(True)
                        
                except (serial.SerialException, OSError, AttributeError):
                    self.is_connected = False
                    self.connection_status.emit(False)
                    
                    try:
                        if self.ser:
                            self.ser.close()
                    except:
                        pass
                    self.ser = None
    
    def _safe_serial_operation(self, operation, *args, **kwargs):
        """
        Execute a serial operation with timeout protection and error recovery
        Returns: (success, result, error_message)
        """
        with self.connection_mutex:
            if not self.is_connected or not self.ser or not self.ser.is_open:
                if not self._find_and_open_port():
                    return False, None, "Not connected to serial port"
            
            try:
                result = operation(*args, **kwargs)
                return True, result, None
                
            except serial.SerialTimeoutException:
                self.is_connected = False
                self.connection_status.emit(False)
                return False, None, "Serial timeout"
                
            except serial.SerialException as e:
                self.is_connected = False
                self.connection_status.emit(False)
                return False, None, f"Serial error: {str(e)}"
                
            except Exception as e:
                return False, None, f"Operation error: {str(e)}"
    
    def _send_command(self, command, expect_response=False, response_size=None):
        """Send command with timeout protection and retry logic"""
        command_bytes = command if isinstance(command, bytes) else command.encode()
        
        # Only clear FIFO for non-measurement commands
        if command_bytes != b'm':
            self._clear_fifo()
        
        for attempt in range(self.command_retry_count + 1):
            if not self.sweep_started and not self.waiting_for_start:
                return False, None, "Sweep stopped"
            
            success, result, error = self._safe_serial_operation(
                self._raw_send_command,
                command_bytes,
                expect_response,
                response_size
            )
            
            if success:
                return True, result, None
            
            if attempt < self.command_retry_count:
                time.sleep(0.1 * (attempt + 1))
        
        return False, None, error or "Command failed after retries"
    
    def _raw_send_command(self, command_bytes, expect_response, response_size):
        """Raw command sending without error handling"""
        # Clear hardware buffers
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        
        # Send command
        self.ser.write(command_bytes)
        self.ser.flush()
        
        time.sleep(0.001)
        
        # Wait for response if needed
        if expect_response and response_size:
            # For measurement command, read directly from serial port
            # (Simpler and more reliable than trying to parse from FIFO)
            return self._read_measurement_directly(response_size)
        
        return None
    
    def _read_measurement_directly(self, size):
        """Read measurement data directly from serial port"""
        # Clear FIFO before measurement
        self._clear_fifo()
        
        # Read directly from serial port
        data = bytearray()
        start_time = time.time()
        
        while len(data) < size:
            if time.time() - start_time > self.read_timeout:
                raise serial.SerialTimeoutException(f"Measurement read timeout after {self.read_timeout} seconds")
            
            bytes_to_read = size - len(data)
            chunk = self.ser.read(min(bytes_to_read, self.ser.in_waiting or 1))
            
            if chunk:
                data.extend(chunk)
            else:
                time.sleep(0.001)
        
        return bytes(data)
    
    def _start_command_thread(self):
        """Start background thread for command processing"""
        self.command_thread_running = True
        self.command_thread = threading.Thread(
            target=self._command_thread_func,
            daemon=True
        )
        self.command_thread.start()
    
    def _command_thread_func(self):
        """Background thread for processing commands"""
        while self.command_thread_running:
            try:
                if self.command_queue:
                    command_info = self.command_queue.popleft()
                    command_type, args, kwargs, callback = command_info
                    
                    try:
                        if command_type == 'measure':
                            result = self._perform_measurement_internal(*args, **kwargs)
                            if callback:
                                callback(result)
                        elif command_type == 'command':
                            success, data, error = self._send_command(*args, **kwargs)
                            if callback:
                                callback(success, data, error)
                    except Exception:
                        pass
                
                time.sleep(0.001)
                
            except Exception:
                time.sleep(0.1)
    
    def _queue_command(self, command_type, callback, *args, **kwargs):
        """Queue a command for background processing"""
        self.command_queue.append((command_type, args, kwargs, callback))
    
    # Public API - COMPATIBLE WITH ORIGINAL
    
    def flush_input(self):
        """Flush serial input buffer - non-blocking"""
        with self.connection_mutex:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                except:
                    pass
        
        # Clear FIFO
        self._clear_fifo()
    
    def start_sweep(self, fi, fo, delta_fw, amplitude=0, start_delay_ms=0, clear_data=True):
        """Start a frequency sweep - COMPATIBLE"""
        self.error_signal.emit("")
        
        # Clear FIFO before starting sweep
        self._clear_fifo()
        
        self.fw = fs(fi)
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
        
        # Queue initial commands
        self._queue_command('command', None, f"ff{self.fw}a\r\n")
        if amplitude > 0:
            time.sleep(0.001)
            self._queue_command('command', None, f"aa{amplitude:03d}a\r\n")
        
        return start_delay_ms
    
    def begin_sweep(self):
        """Begin the actual sweep after delay - COMPATIBLE"""
        self.waiting_for_start = False
        self.sweep_started = True
        return True
    
    def stop_sweep(self):
        """Stop the sweep - COMPATIBLE"""
        self.sweep_started = False
        self.waiting_for_start = False
    
    def perform_measurement(self, fw, amplitude=0, average_count=1, sample_delay_ms=0):
        """
        Perform measurement at a specific frequency word
        COMPATIBLE with original interface - but with timeout protection
        """
        return self._perform_measurement_internal(fw, amplitude, average_count, sample_delay_ms)
    
    def _perform_measurement_internal(self, fw, amplitude=0, average_count=1, sample_delay_ms=0):
        """Internal measurement function with proper error handling"""
        freq_detailed_data = []
        
        for measurement_idx in range(average_count):
            try:
                # Clear FIFO before frequency command
                self._clear_fifo()
                
                # Send frequency command
                success, _, error = self._send_command(f"ff{fw}###")
                if not success:
                    return None, error
                
                time.sleep(0.001)
                
                # Apply sample delay
                if sample_delay_ms > 0:
                    time.sleep(sample_delay_ms / 1000.0)
                
                # Get measurement data
                success, raw, error = self._send_command(b"m", expect_response=True, response_size=1024*4)
                if not success:
                    return None, error
                
                if len(raw) != 1024 * 4:
                    return None, f"Short read: {len(raw)} bytes (expected {1024*4})"
                
                # Process data
                raw_u32 = np.frombuffer(raw, dtype=np.uint32)
                
                if raw_u32.size != 1024:
                    return None, f"Frame error: got {raw_u32.size} words (expected 1024)"
                
                ch0_samples = (raw_u32 & 0xFFFF).astype(np.int16)
                ch1_samples = ((raw_u32 >> 16) & 0xFFFF).astype(np.int16)
                
                ch0 = (ch0_samples - 2048) * 3.3 / 4096
                ch1 = (ch1_samples - 2048) * 3.3 / 4096
                
                freq_hz = fw / (2**28) * 25e6
                
                measurement_data = {
                    'measurement_idx': measurement_idx,
                    'freq_word': fw,
                    'freq_hz': freq_hz,
                    'amplitude': amplitude,
                    'ch0': ch0,
                    'ch1': ch1
                }
                freq_detailed_data.append(measurement_data)
                
            except Exception as e:
                return None, f"Measurement error: {str(e)}"
        
        return freq_detailed_data, None
    
    def perform_measurement_async(self, fw, amplitude=0, average_count=1, sample_delay_ms=0, callback=None):
        """Async version of measurement - won't block GUI"""
        self._queue_command('measure', callback, fw, amplitude, average_count, sample_delay_ms)
    
    def sweep_step(self, delta_fw, amplitude=0, average_count=1, sample_delay_ms=0):
        """Execute one step of the sweep - COMPATIBLE"""
        if self.waiting_for_start or not self.sweep_started:
            return True, None
        
        if self.fw > self.fw_end:
            self.sweep_started = False
            if self.on_sweep_complete:
                self.on_sweep_complete()
            return False, "Done"
        
        # Check if amplitude changed
        if amplitude != self.current_amplitude and amplitude > 0:
            success, _, error = self._send_command(f"aa{amplitude:03d}a\r\n")
            if not success:
                self.sweep_started = False
                return False, f"Amplitude change failed: {error}"
            self.current_amplitude = amplitude
            time.sleep(0.001)
        
        # Perform measurement
        measurements, error = self.perform_measurement(
            self.fw, amplitude, average_count, sample_delay_ms
        )
        
        if error:
            self.sweep_started = False
            self.error_signal.emit(f"Measurement failed: {error}")
            return False, error
        
        # Call sweep step callback
        if self.on_sweep_step and measurements:
            try:
                result = self.on_sweep_step(measurements, self.fw)
                if result:
                    self.detailed_data.append(result['detailed_data'])
            except Exception:
                pass
        
        # Increment frequency
        self.fw += delta_fw
        return True, None
    
    def cleanup(self):
        """Cleanup resources"""
        self.command_thread_running = False
        self.receive_thread_running = False
        self.connection_timer.stop()
        
        if self.command_thread and self.command_thread.is_alive():
            self.command_thread.join(timeout=1.0)
        
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=1.0)
        
        with self.connection_mutex:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                except:
                    pass