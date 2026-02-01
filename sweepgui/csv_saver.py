from datetime import datetime
import re

class CSVSaver:
    def __init__(self, parameters):
        self.parameters = parameters  # Dictionary of measurement parameters
    
    def save_to_csv(self, filename, notes, detailed_data):
        """Save all sweep data to a CSV file"""
        if not detailed_data:
            return "No data to save", False
        
        # Remove any existing .csv extension and add it fresh
        filename = re.sub(r'\.csv$', '', filename) + '.csv'
        
        try:
            with open(filename, 'w', encoding='utf-8') as f:
                # Write comment with notes and parameters - replace line breaks with spaces
                notes_text = notes.strip()
                if notes_text:
                    # Replace line breaks with spaces for CSV compatibility
                    notes_text = notes_text.replace('\n', ' ').replace('\r', ' ')
                    f.write(f"# Notes: {notes_text}\n")
                else:
                    f.write(f"# No note {notes_text}\n")
                
                # Write timestamp
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                f.write(f"# Timestamp: {timestamp}\n")
                
                # Write parameters on a single line
                f.write(f"# Parameters: ")
                params_list = []
                for key, value in self.parameters.items():
                    params_list.append(f"{key}={value}")
                f.write("; ".join(params_list) + "\n")
                f.write("\n")
                
                # Write header
                header = [
                    "freq_word", "freq_hz", "amplitude", "Z_mag_avg", "Z_phase_avg_deg",
                    "measurement_idx", "V_complex_real", "V_complex_imag",
                    "I_complex_real", "I_complex_imag", "A_v", "B_v", "A_i", "B_i",
                    "dc_ch0", "dc_ch1", "rms_residuals_ch0", "rms_residuals_ch1",
                    "voltage_phase_rad"
                ]
                f.write(",".join(header) + "\n")
                
                # Write data
                for freq_data in detailed_data:
                    freq_word = freq_data['freq_word']
                    freq_hz = freq_data['freq_hz']
                    Z_mag_avg = freq_data['Z_mag_avg']
                    Z_phase_avg = freq_data['Z_phase_avg']
                    
                    for measurement in freq_data['measurements']:
                        amplitude = measurement.get('amplitude', 0)
                        row = [
                            str(freq_word),
                            f"{freq_hz:.6f}",
                            str(amplitude),
                            f"{Z_mag_avg:.6f}",
                            f"{Z_phase_avg:.6f}",
                            str(measurement['measurement_idx']),
                            f"{measurement['V_complex_real']:.6f}",
                            f"{measurement['V_complex_imag']:.6f}",
                            f"{measurement['I_complex_real']:.6f}",
                            f"{measurement['I_complex_imag']:.6f}",
                            f"{measurement['A_v']:.6f}",
                            f"{measurement['B_v']:.6f}",
                            f"{measurement['A_i']:.6f}",
                            f"{measurement['B_i']:.6f}",
                            f"{measurement['dc_ch0']:.6f}",
                            f"{measurement['dc_ch1']:.6f}",
                            f"{measurement['rms_residuals_ch0']:.6f}",
                            f"{measurement['rms_residuals_ch1']:.6f}",
                            f"{measurement['voltage_phase_rad']:.6f}"
                        ]
                        f.write(",".join(row) + "\n")
            
            return f"Data saved to {filename}", True
            
        except Exception as e:
            return f"Error saving file: {str(e)}", False