import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore

class PlotManager:
    def __init__(self):
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.setup_plots()
        
        # Plot dummy data to fix legend colors on startup
        QtCore.QTimer.singleShot(50, self.plot_dummy_data)
        
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
        self.plot2.setYRange(4, 9)  # Ω range
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
    
    def plot_dummy_data(self):
        """Plot dummy data to fix legend colors on startup"""
        # Plot a single invisible point
        self.curve_mag.setData([1], [1])
        self.curve_phase.setData([1], [1])
        
        # Clear it immediately
        self.curve_mag.setData([], [])
        self.curve_phase.setData([], [])
    
    def update_views(self):
        """Update phase ViewBox geometry when main plot changes"""
        self.phase_vb.setGeometry(self.plot2.vb.sceneBoundingRect())
        self.phase_vb.linkedViewChanged(self.plot2.vb, self.phase_vb.XAxis)
    
    def update_oscilloscope(self, t_aligned, ch0, ch1, fit_params=None):
        """Update oscilloscope plot
        fit_params: Dictionary with A_v, B_v, C_v, A_i, B_i, C_i from impedance calculator
        """
        # Update plots
        self.curve0.setData(t_aligned, ch0)
        self.curve1.setData(t_aligned, ch1)
        
        # Calculate Y range based on fitted sine wave parameters
        if fit_params is not None:
            # Get fitted parameters
            A_v = fit_params.get('A_v', 0)
            B_v = fit_params.get('B_v', 0)
            C_v = fit_params.get('C_v', 0)
            A_i = fit_params.get('A_i', 0)
            B_i = fit_params.get('B_i', 0)
            C_i = fit_params.get('C_i', 0)
            
            # Calculate amplitude for each channel
            # Signal: A*cos(ωt) + B*sin(ωt) + C
            # Amplitude = sqrt(A² + B²)
            amp_v = np.sqrt(A_v**2 + B_v**2)
            amp_i = np.sqrt(A_i**2 + B_i**2)
            
            # Calculate min and max for each channel based on fitted sine wave
            ch0_min_fit = C_v - amp_v
            ch0_max_fit = C_v + amp_v
            ch1_min_fit = C_i - amp_i
            ch1_max_fit = C_i + amp_i
            
            # Get overall range across both channels
            overall_min = min(ch0_min_fit, ch1_min_fit)
            overall_max = max(ch0_max_fit, ch1_max_fit)
            
            # Add padding
            padding = max(amp_v, amp_i) * 0.1  # 10% of the larger amplitude
            y_min = overall_min - padding
            y_max = overall_max + padding
            
            # Set Y range
            self.plot.setYRange(y_min, y_max, padding=0)
        else:
            # Fallback: use data min/max if no fit parameters provided
            ch0_min = np.min(ch0)
            ch0_max = np.max(ch0)
            ch1_min = np.min(ch1)
            ch1_max = np.max(ch1)
            
            overall_min = min(ch0_min, ch1_min)
            overall_max = max(ch0_max, ch1_max)
            
            if overall_min != overall_max:
                padding = (overall_max - overall_min) * 0.1
                y_min = overall_min - padding
                y_max = overall_max + padding
            else:
                y_min = overall_min - 0.1
                y_max = overall_max + 0.1
            
            self.plot.setYRange(y_min, y_max, padding=0)
    
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