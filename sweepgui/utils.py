import numpy as np
import re
import os

def fs(f):
    """Convert frequency to frequency word (same as Octave helper)"""
    return int(round(f * (2**28) / 25e6))

def format_frequency(freq_hz):
    """Format frequency with auto-scaling"""
    if freq_hz >= 1e6:
        return f"{freq_hz/1e6:.5f} MHz"
    elif freq_hz >= 1e3:
        return f"{freq_hz/1e3:.4f} kHz"
    else:
        return f"{freq_hz:.1f} Hz"

def format_impedance(z_mag):
    """Format impedance with auto-scaling"""
    if z_mag >= 1e6:
        return f"{z_mag/1e6:.2f} MΩ"
    elif z_mag >= 1e3:
        return f"{z_mag/1e3:.2f} kΩ"
    else:
        return f"{z_mag:.2f} Ω"

def generate_new_filename(current_filename):
    """Generate a new filename with auto-numbering based on existing files"""
    # Extract base name and existing counter
    match = re.match(r'^(.*?)(?:_(\d{3}))?\.csv$', current_filename)
    
    if match:
        base_name = match.group(1)
        existing_counter = match.group(2)
        
        if existing_counter:
            # Start from existing counter + 1
            counter = int(existing_counter) + 1
            base_name = base_name.rstrip('_')  # Remove trailing underscore if present
        else:
            # No counter found, start from 001
            counter = 1
    else:
        # No .csv extension or pattern doesn't match
        base_name = re.sub(r'\.csv$', '', current_filename)
        if not base_name:
            base_name = "sweep_data"
        counter = 1
    
    # Find the next available number if file exists
    while os.path.exists(f"{base_name}_{counter:03d}.csv"):
        counter += 1
    
    return f"{base_name}_{counter:03d}.csv"