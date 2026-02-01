#!/usr/bin/env python3
"""
Main application entry point for Impedance Analyzer.
Sets up the application, configures theme, and starts the main loop.
"""

import sys
import signal
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QPalette, QColor
from PyQt5.QtCore import Qt, QTimer
import pyqtgraph as pg

from sweep_gui import SweepGUI

def configure_dark_theme(app):
    """Configure dark theme for the application"""
    # Set Fusion style which works well with dark themes
    app.setStyle('Fusion')
    
    # Create and apply DARKER color palette
    dark_palette = QPalette()
    
    # Base colors - much darker
    dark_palette.setColor(QPalette.Window, QColor(30, 30, 30))        # Dark gray background
    dark_palette.setColor(QPalette.WindowText, QColor(220, 220, 220)) # Light gray text
    dark_palette.setColor(QPalette.Base, QColor(18, 18, 18))          # Very dark for input fields
    dark_palette.setColor(QPalette.AlternateBase, QColor(35, 35, 35)) # Slightly lighter for alternating rows
    
    # Text colors
    dark_palette.setColor(QPalette.Text, QColor(240, 240, 240))       # Very light gray for text
    dark_palette.setColor(QPalette.BrightText, QColor(255, 50, 50))   # Red for bright text
    
    # Button colors
    dark_palette.setColor(QPalette.Button, QColor(40, 40, 40))        # Dark buttons
    dark_palette.setColor(QPalette.ButtonText, QColor(220, 220, 220)) # Light text on buttons
    
    # Tooltip colors
    dark_palette.setColor(QPalette.ToolTipBase, QColor(20, 20, 20))   # Very dark tooltip
    dark_palette.setColor(QPalette.ToolTipText, QColor(240, 240, 240))# Light tooltip text
    
    # Selection/Highlight colors
    dark_palette.setColor(QPalette.Highlight, QColor(42, 130, 218))   # Blue highlight
    dark_palette.setColor(QPalette.HighlightedText, QColor(255, 255, 255)) # White for highlighted text
    
    # Link color
    dark_palette.setColor(QPalette.Link, QColor(42, 130, 218))        # Blue links
    
    # Disabled elements
    dark_palette.setColor(QPalette.Disabled, QPalette.Text, QColor(100, 100, 100))
    dark_palette.setColor(QPalette.Disabled, QPalette.ButtonText, QColor(100, 100, 100))
    
    app.setPalette(dark_palette)
    
    # Configure pyqtgraph for even darker theme
    pg.setConfigOption('background', (2, 2, 2))  # Very dark background for plots
    # pg.setConfigOption('foreground', 'w')           # White foreground
    # pg.setConfigOption('antialias', True)

def setup_application():
    """Setup and configure the QApplication"""
    app = QApplication(sys.argv)
    app.setApplicationName("Serial Sweep")
    app.setOrganizationName("Impedance Analyzer")
    
    # Configure theme
    configure_dark_theme(app)
    
    return app

def signal_handler(sig, frame):
    """Handle Ctrl+C signal"""
    print("\nCtrl+C detected. Closing application...")
    QApplication.quit()

def main():
    """Main application entry point"""
    # Setup application
    app = setup_application()
    
    # Install signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    
    # Create a timer to periodically process Python signals
    # This allows Ctrl+C to work during the Qt event loop
    timer = QTimer()
    timer.start(500)  # Check every 500ms
    timer.timeout.connect(lambda: None)  # Just to let Python process signals
    
    # Create and show main window
    main_window = SweepGUI()
    main_window.show()
    
    # Force layout calculation and redraw
    QApplication.processEvents()
    main_window.plot_manager.plot_widget.ci.layout.activate()
    main_window.plot_manager.update_views()
    
    # Start event loop
    return_code = app.exec_()
    
    # Cleanup and exit
    sys.exit(return_code)

if __name__ == "__main__":
    main()