import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore

class PlotManager:
    def __init__(self):
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.setup_plots()
        
    def setup_plots(self):
        """Setup oscilloscope and analyzer plots"""
        # Oscilloscope plot
        self.plot = self.plot_widget.addPlot(title="Oscilloscope")
        self.plot.showGrid(x=True, y=True)
        self.plot.setLabel('left', 'Amplitude', units='V')
        self.plot.setLabel('bottom', 'Time', units='μs')
        self.plot.getAxis('bottom').enableAutoSIPrefix(False)
        
        self.plot_widget.nextRow()
        
        # Impedance analyzer plot
        self.plot2 = self.plot_widget.addPlot(title="Impedance Analyzer")
        self.plot2.showGrid(x=True, y=True)
        self.plot2.setLogMode(y=True, x=False)
        self.plot2.setLabel('left', 'Impedance', units='Ω')
        self.plot2.setLabel('bottom', 'Frequency', units='Hz')
        self.plot2.getAxis('left').enableAutoSIPrefix(False)
        
        # Set reasonable default ranges
        self.plot2.setYRange(4, 8)  # Ω range
        self.plot2.setXRange(32720, 32800)  # Hz range
        
        # Add right axis for phase
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
        
        # Set layout stretch factors
        self.plot_widget.ci.layout.setRowStretchFactor(0, 3)  # Oscilloscope: 30%
        self.plot_widget.ci.layout.setRowStretchFactor(1, 7)  # Analyzer: 70%
        
        # Create curves
        self.curve0 = self.plot.plot(pen='y', name="CH0")
        self.curve1 = self.plot.plot(pen='c', name="CH1")
        self.curve_mag = self.plot2.plot(pen='r', symbol='o', symbolPen='r', 
                                        symbolBrush='r', symbolSize=5, name="|Z|")
        
        # Create phase curve with explicit connect mode
        self.curve_phase = pg.PlotCurveItem(
            pen='b', 
            symbol='s', 
            symbolPen='b',
            symbolBrush='b', 
            symbolSize=5, 
            name="Phase",
            connect='finite'  # Explicitly set to break at nan/inf
        )
        self.phase_vb.addItem(self.curve_phase)
        
        # Legend
        self.legend = pg.LegendItem(offset=(70, 30))
        self.legend.setParentItem(self.plot2)
        self.legend.addItem(self.curve_mag, 'Impedance (Ω)')
        self.legend.addItem(self.curve_phase, 'Phase (°)')
        
        # Connect plot resize
        self.plot2.sigRangeChanged.connect(self.update_views)
    
    def update_views(self):
        """Update phase ViewBox geometry when main plot changes"""
        self.phase_vb.setGeometry(self.plot2.vb.sceneBoundingRect())
        self.phase_vb.linkedViewChanged(self.plot2.vb, self.phase_vb.XAxis)
    
    def update_oscilloscope(self, t_aligned, ch0, ch1):
        """Update oscilloscope plot"""
        self.curve0.setData(t_aligned, ch0)
        self.curve1.setData(t_aligned, ch1)
    
    def update_analyzer(self, frequencies, magnitudes, phases):
        """Update impedance analyzer plot"""
        if len(frequencies) > 0:
            # Convert to numpy arrays for plotting
            freqs_np = np.array(frequencies)
            mags_np = np.array(magnitudes)
            phases_np = np.array(phases)
            
            # Plot ALL data including nan values
            # pyqtgraph will automatically break lines at nan values
            # and won't draw symbols at nan points
            self.curve_mag.setData(freqs_np, mags_np)
            self.curve_phase.setData(freqs_np, phases_np)
            
    def clear_plots(self):
        """Clear all plot data"""
        self.curve_mag.setData([], [])
        self.curve_phase.setData([], [])
    
    def set_xrange(self, fi, fo):
        """Set X range for analyzer plot"""
        self.plot2.setXRange(fi, fo)