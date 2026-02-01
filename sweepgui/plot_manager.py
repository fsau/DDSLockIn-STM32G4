import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore

class PlotManager:
    def __init__(self):
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.setup_plots()
        # Track sweeps separately for hold mode
        self.current_sweep_freq = []
        self.current_sweep_mag = []
        self.current_sweep_phase = []
        self.all_sweeps_freq = []
        self.all_sweeps_mag = []
        self.all_sweeps_phase = []
        
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
        self.curve_phase = pg.PlotCurveItem(pen='b', symbol='s', symbolPen='b',
                                           symbolBrush='b', symbolSize=5, name="Phase")
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
            
            # Filter out invalid values
            valid_idx = (mags_np > 0) & np.isfinite(mags_np) & np.isfinite(phases_np)
            
            if np.any(valid_idx):
                self.curve_mag.setData(freqs_np[valid_idx], mags_np[valid_idx])
                self.curve_phase.setData(freqs_np[valid_idx], phases_np[valid_idx])
    
    def clear_plots(self):
        """Clear all plot data"""
        self.curve_mag.setData([], [])
        self.curve_phase.setData([], [])
        # Also clear sweep tracking
        self.current_sweep_freq.clear()
        self.current_sweep_mag.clear()
        self.current_sweep_phase.clear()
        self.all_sweeps_freq.clear()
        self.all_sweeps_mag.clear()
        self.all_sweeps_phase.clear()
    
    def set_xrange(self, fi, fo):
        """Set X range for analyzer plot"""
        self.plot2.setXRange(fi, fo)
        
    def add_measurement(self, frequency, magnitude, phase, is_new_sweep=False):
        """Add a single measurement point to the plot (for hold mode)"""
        if magnitude > 0 and np.isfinite(magnitude) and np.isfinite(phase):
            # Start new sweep if requested
            if is_new_sweep:
                # Save current sweep to all sweeps
                if self.current_sweep_freq:
                    self.all_sweeps_freq.append(np.array(self.current_sweep_freq))
                    self.all_sweeps_mag.append(np.array(self.current_sweep_mag))
                    self.all_sweeps_phase.append(np.array(self.current_sweep_phase))
                # Start new sweep
                self.current_sweep_freq = []
                self.current_sweep_mag = []
                self.current_sweep_phase = []
            
            # Add to current sweep
            self.current_sweep_freq.append(frequency)
            self.current_sweep_mag.append(magnitude)
            self.current_sweep_phase.append(phase)
            
            # Combine all sweeps for plotting
            all_freq = []
            all_mag = []
            all_phase = []
            
            for i, sweep_freq in enumerate(self.all_sweeps_freq):
                all_freq.extend(sweep_freq)
                all_mag.extend(self.all_sweeps_mag[i])
                all_phase.extend(self.all_sweeps_phase[i])
            
            # Add current sweep
            if self.current_sweep_freq:
                all_freq.extend(self.current_sweep_freq)
                all_mag.extend(self.current_sweep_mag)
                all_phase.extend(self.current_sweep_phase)
            
            # Update plot
            if all_freq:
                self.curve_mag.setData(np.array(all_freq), np.array(all_mag))
                self.curve_phase.setData(np.array(all_freq), np.array(all_phase))