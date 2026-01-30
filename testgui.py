import sys
import time
import numpy as np
import serial
import serial.tools.list_ports
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore


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
        
        self.plot2 = self.plot_widget.addPlot(title="Analyzer")
        self.plot2.showGrid(x=True, y=True)
        self.plot2.setLogMode(y=True)
        self.plot2.setYRange(2,8)
        self.plot2.setLabel('left', 'Impedance', units='ohm')
        self.plot2.setLabel('bottom', 'Frequency', units='Hz')
        self.axis_right = pg.AxisItem('right')
        self.axis_right.setLabel('Phase', units='°')
        self.axis_right.setRange(-100,100)
        self.plot2.layout.addItem(self.axis_right, 2, 3)

        self.curve0 = self.plot.plot(pen='y', name="CH0")
        self.curve1 = self.plot.plot(pen='c', name="CH1")

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

        self.start_btn = QtWidgets.QPushButton("Start / Update")
        self.status = QtWidgets.QLabel("Idle")

        form.addRow("fi [Hz]:", self.fi_box)
        form.addRow("fo [Hz]:", self.fo_box)
        form.addRow(self.start_btn)
        form.addRow("Status:", self.status)

        # ---------------- Sweep state ----------------
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.sweep_step)

        self.fw = 0
        self.fw_end = 0

        self.start_btn.clicked.connect(self.start_sweep)

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

        self.plot2.setXRange(fi,fo)
        
        if fo < fi:
            self.status.setText("fo < fi")
            return

        self.flush_input()

        self.fw = fs(fi)
        self.fw_end = fs(fo)

        self.status.setText("Sweeping...")
        self.timer.start(1)   # 1 ms per step (matches your pause)

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

        # ----- update plot -----
        self.curve0.setData(ch0)
        self.curve1.setData(ch1)

        freq_khz = self.fw / (2**28) * 25e3
        self.status.setText(f"fw = {freq_khz:.2f} kHz")

        self.fw += 1


# ------------------------------------------------------------
# Run
# ------------------------------------------------------------
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    gui = SweepGUI()
    gui.show()
    sys.exit(app.exec())
