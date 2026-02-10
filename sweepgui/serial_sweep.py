import time
import serial
import serial.tools.list_ports
import numpy as np
from PyQt5.QtCore import QTimer, pyqtSignal, QObject
import threading
from collections import deque
import queue
import struct

from utils import fs

class SerialSweep(QObject):
    # Qt signals
    data_ready = pyqtSignal(object)
    error_signal = pyqtSignal(str)
    connection_status = pyqtSignal(bool)

    # ------------------------------------------------------------------
    # INIT
    # ------------------------------------------------------------------
    def __init__(self):
        super().__init__()

        # ---------------- Serial state ----------------
        self.ser = None
        self.port_name = None
        self.is_connected = False
        self.connection_mutex = threading.RLock()

        self.baudrate = 115200
        self.read_timeout = 2.0
        self.write_timeout = 0.5
        self.command_retry_count = 2

        # ---------------- Protocol ----------------
        self.MEASUREMENT_HEADER_ADC = b'\x55\x55\x55\x00'
        self.MEASUREMENT_HEADER_AUX = b'\x55\x55\x55\x10'
        self.MEASUREMENT_HEADER_DEMOD = b'\x55\x55\x55\x20'  # NEW: Demodulated data
        self.DEMOD_END_MARKER = b'\xAA'  # NEW: End of demod data frame

        self.MEASUREMENT_DATA_SIZE = 1024 * 4  # ADC
        self.AUX_DATA_SIZE = 20
        # NEW: Size of ddsli_output_t struct in bytes
        # phase (8) + phase_inc (8) + phase_inc_delta (8) + A1 (2) + A2 (2) + chA[3] (12) + chB[3] (12)
        self.DEMOD_OUTPUT_SIZE = 8 + 8 + 8 + 2 + 2 + 12 + 12  # = 52 bytes

        self.FRAME_DEFS = {
            self.MEASUREMENT_HEADER_ADC: {
                'name': 'adc',
                'payload': self.MEASUREMENT_DATA_SIZE
            },
            self.MEASUREMENT_HEADER_AUX: {
                'name': 'aux',
                'payload': self.AUX_DATA_SIZE
            },
            # NEW: Demod data frame definition
            self.MEASUREMENT_HEADER_DEMOD: {
                'name': 'demod',
                'payload': 0  # Variable length, will parse specially
            }
        }

        self.HEADER_SIZE = 4
        self.MAX_FRAME_SIZE = max(
            self.HEADER_SIZE + d['payload']
            for d in self.FRAME_DEFS.values()
        )

        # ---------------- FIFOs ----------------
        self.rx_byte_fifo = queue.Queue(maxsize=200_000)

        self.adc_fifo = queue.Queue(maxsize=64)
        self.aux_fifo = queue.Queue(maxsize=128)
        self.cmd_fifo = queue.Queue(maxsize=128)
        self.demod_fifo = queue.Queue(maxsize=256)  # NEW: For demodulated data

        # ---------------- Threads ----------------
        self.rx_thread_running = False
        self.rx_thread = None

        self.parser_running = False
        self.parser_thread = None

        self.command_queue = deque()
        self.command_thread_running = False
        self.command_thread = None

        # ---------------- Sweep state ----------------
        self.fw = 0
        self.fw_end = 0
        self.sweep_started = False
        self.waiting_for_start = False
        self.current_amplitude = 0

        self.frequencies = []
        self.impedance_magnitudes = []
        self.phases = []
        self.detailed_data = []

        self.on_sweep_step = None
        self.on_sweep_complete = None

        # ---------------- Startup ----------------
        self._find_and_open_port()

        # self.connection_timer = QTimer()
        # self.connection_timer.timeout.connect(self._check_connection)
        # self.connection_timer.start(2000)

        self._start_rx_thread()
        self._start_parser_thread()
        self._start_command_thread()

    # ------------------------------------------------------------------
    # SERIAL RX THREAD (RAW BYTES ONLY)
    # ------------------------------------------------------------------
    def _start_rx_thread(self):
        self.rx_thread_running = True
        self.rx_thread = threading.Thread(
            target=self._rx_thread_func,
            daemon=True,
            name="SerialRX"
        )
        self.rx_thread.start()

    def _rx_thread_func(self):
        while self.rx_thread_running:
            try:
                with self.connection_mutex:
                    if not self.is_connected or not self.ser or not self.ser.is_open:
                        time.sleep(0.05)
                        continue

                n = self.ser.in_waiting
                if n:
                    data = self.ser.read(n)
                    for b in data:
                        try:
                            self.rx_byte_fifo.put_nowait(b)
                        except queue.Full:
                            pass
                else:
                    time.sleep(0.001)

            except serial.SerialException:
                with self.connection_mutex:
                    self.is_connected = False
                    self.connection_status.emit(False)
                time.sleep(0.1)

    # ------------------------------------------------------------------
    # FRAME PARSER THREAD
    # ------------------------------------------------------------------
    def _start_parser_thread(self):
        self.parser_running = True
        self.parser_thread = threading.Thread(
            target=self._parser_thread_func,
            daemon=True,
            name="SerialParser"
        )
        self.parser_thread.start()

    def _parser_thread_func(self):
        buf = bytearray()

        while self.parser_running:
            try:
                b = self.rx_byte_fifo.get(timeout=0.1)
                buf.append(b)

                # Limit buffer size
                if len(buf) > 2 * self.MAX_FRAME_SIZE:
                    buf = buf[-self.MAX_FRAME_SIZE:]

                while True:
                    if len(buf) < self.HEADER_SIZE:
                        break

                    header = bytes(buf[:4])

                    # ASCII / command response fallback
                    if header[:1] != b'\x55' and len(buf) < 64 and buf.endswith(b'\n'):
                        self._fifo_put(self.cmd_fifo, bytes(buf))
                        buf.clear()
                        break

                    # NEW: Special handling for demod data
                    if header == self.MEASUREMENT_HEADER_DEMOD:
                        # Find the end marker (0xAA) to determine frame length
                        try:
                            end_index = buf.find(self.DEMOD_END_MARKER, 4)
                            if end_index == -1:
                                # Haven't received the end marker yet
                                break
                            
                            # Extract the entire demod frame
                            frame_data = bytes(buf[:end_index + 1])  # Include the 0xAA
                            del buf[:end_index + 1]
                            
                            # Process the demod frame
                            self._process_demod_frame(frame_data)
                            continue  # Continue parsing buffer
                            
                        except Exception as e:
                            # If parsing fails, skip the header byte and continue
                            buf.pop(0)
                            continue
                    
                    # Original frame handling
                    if header not in self.FRAME_DEFS:
                        buf.pop(0)
                        continue

                    frame = self.FRAME_DEFS[header]
                    total_len = self.HEADER_SIZE + frame['payload']

                    if len(buf) < total_len:
                        break

                    payload = bytes(buf[4:total_len])
                    del buf[:total_len]

                    if frame['name'] == 'adc':
                        self._fifo_put(self.adc_fifo, payload)
                    elif frame['name'] == 'aux':
                        self._fifo_put(self.aux_fifo, payload)

            except queue.Empty:
                pass

    def _process_demod_frame(self, frame_data):
        """
        Process a demodulated data frame.
        Format: 0x55 0x55 0x55 0x20 [output 1] 0x77 [output 2] 0x77 ... 0x77 [output N] 0xAA
        where each output is ddsli_output_t (52 bytes)
        """
        # Skip the header (4 bytes) and end marker (1 byte)
        if len(frame_data) < 5:
            return
        
        # Extract the data between header and end marker
        data_section = frame_data[4:-1]  # Remove header and 0xAA
        
        # Parse multiple ddsli_output_t structures
        outputs = []
        pos = 0
        
        print("b:", frame_data[0:70])
        while pos < len(data_section):
            # Each output starts with 0x77 separator
            if data_section[pos] != 0x77:
                # Malformed data, skip to next 0x77
                try:
                    next_pos = data_section.find(b'\x77', pos)
                    if next_pos == -1:
                        break
                    pos = next_pos
                    continue
                except:
                    break
            
            # Skip the 0x77 separator
            pos += 1
            
            # Check if we have enough bytes for a ddsli_output_t
            if pos + self.DEMOD_OUTPUT_SIZE > len(data_section):
                break
            
            # Parse ddsli_output_t structure
            try:
                output = self._parse_ddsli_output(data_section[pos:pos + self.DEMOD_OUTPUT_SIZE])
                if output:
                    outputs.append(output)
            except Exception as e:
                # If parsing fails, skip this output
                pass
            
            pos += self.DEMOD_OUTPUT_SIZE
        
        # Put all parsed outputs into the demod FIFO
        if outputs:
            try:
                self.demod_fifo.put_nowait(outputs)
            except queue.Full:
                try:
                    self.demod_fifo.get_nowait()
                    self.demod_fifo.put_nowait(outputs)
                except queue.Empty:
                    pass

    def _parse_ddsli_output(self, data):
        """
        Parse a single ddsli_output_t structure (52 bytes)
        """
        try:
            # Unpack the structure
            # Format: < phase (q) phase_inc (q) phase_inc_delta (q) A1 (h) A2 (h) chA[3] (3f) chB[3] (3f)
            # Using little-endian byte order
            unpacked = struct.unpack('<qqqhhffffffff', data)
            
            # Create structured output
            output = {
                'frequency': {
                    'phase': unpacked[0],           # Q32.32
                    'phase_inc': unpacked[1],       # Q32.32  
                    'phase_inc_delta': unpacked[2]  # Q32.32
                },
                'ddsli_amplitude': {
                    'A1': unpacked[3],              # Q1.15
                    'A2': unpacked[4]               # Q1.15
                },
                'chA': [unpacked[5], unpacked[6], unpacked[7]],  # I, Q, DC (float)
                'chB': [unpacked[8], unpacked[9], unpacked[10]]  # I, Q, DC (float)
            }
            
            return output
            
        except struct.error:
            return None

    def _fifo_put(self, fifo, item):
        try:
            fifo.put_nowait(item)
        except queue.Full:
            try:
                fifo.get_nowait()
                fifo.put_nowait(item)
            except queue.Empty:
                pass

    # ------------------------------------------------------------------
    # NEW: DEMOD DATA READING
    # ------------------------------------------------------------------
    def read_demod_data(self):
        """
        Read available demodulated data from FIFO.
        Returns list of ddsli_output_t structures or empty list if none available.
        """
        try:
            return self.demod_fifo.get_nowait()
        except queue.Empty:
            return []

    def request_demod_data(self):
        """
        Send 'p' command to request demodulated data.
        """
        ok, _, err = self._send_command(b'p')
        return ok, err

    # ------------------------------------------------------------------
    # CONNECTION MANAGEMENT
    # ------------------------------------------------------------------
    def _find_and_open_port(self):
        with self.connection_mutex:
            try:
                ports = list(serial.tools.list_ports.comports())
                if not ports:
                    self.is_connected = False
                    self.connection_status.emit(False)
                    return False

                self.port_name = ports[-1].device
                self.ser = serial.Serial(
                    self.port_name,
                    self.baudrate,
                    timeout=self.read_timeout,
                    write_timeout=self.write_timeout
                )

                self.is_connected = True
                self.connection_status.emit(True)
                return True

            except Exception:
                self.is_connected = False
                self.connection_status.emit(False)
                return False

    def _check_connection(self):
        with self.connection_mutex:
            try:
                if not self.ser or not self.ser.is_open:
                    self.is_connected = False
                    self.connection_status.emit(False)
            except Exception:
                self.is_connected = False
                self.connection_status.emit(False)

    # ------------------------------------------------------------------
    # SERIAL COMMANDS
    # ------------------------------------------------------------------
    def _safe_serial(self, fn, *a):
        with self.connection_mutex:
            if not self.is_connected:
                return False, None, "Not connected"
            try:
                return True, fn(*a), None
            except Exception as e:
                return False, None, str(e)

    def _send_command(self, cmd, expect_response=False):
        if isinstance(cmd, str):
            cmd = cmd.encode()

        for _ in range(self.command_retry_count + 1):
            ok, _, err = self._safe_serial(self._raw_send, cmd)
            if ok:
                if expect_response:
                    return True, self._read_command_response(), None
                return True, None, None
            time.sleep(0.1)

        return False, None, err

    def _raw_send(self, cmd):
        self.ser.reset_input_buffer()
        self.ser.write(cmd)
        self.ser.flush()

    def _read_command_response(self):
        try:
            return self.cmd_fifo.get(timeout=1.0)
        except queue.Empty:
            raise serial.SerialTimeoutException("Command timeout")

    # ------------------------------------------------------------------
    # MEASUREMENT READ (ADC FIFO ONLY)
    # ------------------------------------------------------------------
    def _read_measurement_directly(self):
        try:
            data = self.adc_fifo.get(timeout=self.read_timeout)
        except queue.Empty:
            raise serial.SerialTimeoutException("ADC frame timeout")

        if len(data) != self.MEASUREMENT_DATA_SIZE:
            raise serial.SerialTimeoutException("ADC frame size error")

        return data

    # ------------------------------------------------------------------
    # COMMAND THREAD
    # ------------------------------------------------------------------
    def _start_command_thread(self):
        self.command_thread_running = True
        self.command_thread = threading.Thread(
            target=self._command_thread_func,
            daemon=True,
            name="CommandThread"
        )
        self.command_thread.start()

    def _command_thread_func(self):
        while self.command_thread_running:
            if self.command_queue:
                kind, args, cb = self.command_queue.popleft()
                if kind == 'measure':
                    res = self._perform_measurement_internal(*args)
                    if cb:
                        cb(res)
            time.sleep(0.001)

    # ------------------------------------------------------------------
    # PUBLIC API (UNCHANGED)
    # ------------------------------------------------------------------
    def start_sweep(self, fi, fo, delta_fw, amplitude=0, start_delay_ms=0, clear_data=True):
        self.fw = fs(fi)
        self.fw_end = fs(fo)
        self.current_amplitude = amplitude

        if clear_data:
            self.frequencies.clear()
            self.impedance_magnitudes.clear()
            self.phases.clear()
            self.detailed_data.clear()

        self.waiting_for_start = True
        self.sweep_started = False

        self._send_command(f"ff{self.fw}a\r\n")
        if amplitude:
            self._send_command(f"aa{amplitude:03d}a\r\n")

        return start_delay_ms

    def begin_sweep(self):
        self.waiting_for_start = False
        self.sweep_started = True
        return True

    def stop_sweep(self):
        self.sweep_started = False
        self.waiting_for_start = False

    # ------------------------------------------------------------------
    # MEASUREMENT
    # ------------------------------------------------------------------
    def perform_measurement(self, fw, amplitude=0, average_count=1, sample_delay_ms=0):
        return self._perform_measurement_internal(fw, amplitude, average_count, sample_delay_ms)

    def _perform_measurement_internal(self, fw, amplitude, avg, delay):
        results = []

        for i in range(avg):
            self._send_command(f"ff{fw}###")
            if delay:
                time.sleep(delay / 1000)

            self._send_command(b"m")
            raw = self._read_measurement_directly()

            raw_u32 = np.frombuffer(raw, dtype=np.uint32)

            ch0_samples = (raw_u32 & 0xFFFF).astype(np.int16)
            ch1_samples = ((raw_u32 >> 16) & 0xFFFF).astype(np.int16)

            ch0 = (ch0_samples - 2048) * 3.3 / 4096
            ch1 = (ch1_samples - 2048) * 3.3 / 4096

            freq_hz = fw / (2**28) * 25e6

            results.append({
                'measurement_idx': i,
                'freq_word': fw,
                'freq_hz': freq_hz,
                'amplitude': amplitude,
                'ch0': ch0,
                'ch1': ch1
            })

        return results, None

    def sweep_step(self, delta_fw, amplitude=0, average_count=1, sample_delay_ms=0):
        """
        Execute one step of the sweep.
        API-compatible with the original implementation.
        """

        if self.waiting_for_start or not self.sweep_started:
            return True, None

        if self.fw > self.fw_end:
            self.sweep_started = False
            if self.on_sweep_complete:
                self.on_sweep_complete()
            return False, "Done"

        # Update amplitude if needed
        if amplitude != self.current_amplitude and amplitude > 0:
            ok, _, err = self._send_command(f"aa{amplitude:03d}a\r\n")
            if not ok:
                self.sweep_started = False
                return False, f"Amplitude change failed: {err}"
            self.current_amplitude = amplitude
            time.sleep(0.001)

        # Perform measurement
        measurements, error = self.perform_measurement(
            self.fw,
            amplitude,
            average_count,
            sample_delay_ms
        )

        if error:
            self.sweep_started = False
            self.error_signal.emit(f"Measurement failed: {error}")
            return False, error

        # Callback to GUI / processing layer
        if self.on_sweep_step and measurements:
            try:
                result = self.on_sweep_step(measurements, self.fw)
                if result:
                    self.detailed_data.append(result['detailed_data'])
            except Exception:
                pass

        # Advance frequency
        self.fw += delta_fw
        return True, None
    
    # ------------------------------------------------------------------
    # CLEANUP
    # ------------------------------------------------------------------
    def cleanup(self):
        self.rx_thread_running = False
        self.parser_running = False
        self.command_thread_running = False

        if self.rx_thread:
            self.rx_thread.join(timeout=1)
        if self.parser_thread:
            self.parser_thread.join(timeout=1)
        if self.command_thread:
            self.command_thread.join(timeout=1)

        with self.connection_mutex:
            if self.ser and self.ser.is_open:
                self.ser.close()

    def request_demod_measurements(self):
        """Send 'p' command to request demodulated data and return all available measurements."""
        # Send the 'p' command
        ok, err = self.request_demod_data()
        if not ok:
            return [], f"Failed to request demod data: {err}"
        
        # Read all available demod data
        all_demod_outputs = []
        max_reads = 10  # Safety limit
        
        for _ in range(max_reads):
            demod_data = self.read_demod_data()
            if demod_data:
                all_demod_outputs.extend(demod_data)
            else:
                break
        
        return all_demod_outputs, None

    def _phase_inc_to_freq(self, phase_inc):
        """Convert Q32.32 phase increment to frequency in Hz."""
        # phase_inc is Q32.32 fixed point
        # Frequency = phase_inc * f_clk / 2^32
        # Where f_clk = 25 MHz
        freq_hz = (phase_inc / (2**32)) * 25e6
        return freq_hz

    def _q1_15_to_float(self, q_value):
        """Convert Q1.15 fixed-point to float."""
        return q_value / 32768.0