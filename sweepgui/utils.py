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
    import glob
    
    # Extract base name from current filename
    # Remove .csv extension and any trailing _XXX pattern
    base_name = re.sub(r'\.csv$', '', current_filename)
    base_name = re.sub(r'_(\d{3})$', '', base_name)
    
    if not base_name:
        base_name = "sweep_data"
    
    # Find all existing files with this base name
    existing_files = glob.glob(f"{base_name}_*.csv")
    
    if not existing_files:
        # No files exist, start from 001
        return f"{base_name}_001.csv"
    
    # Extract counters from existing files and find the maximum
    max_counter = 0
    for file in existing_files:
        match = re.search(r'_(\d{3})\.csv$', file)
        if match:
            counter = int(match.group(1))
            if counter > max_counter:
                max_counter = counter
    
    # Use next number
    return f"{base_name}_{max_counter + 1:03d}.csv"