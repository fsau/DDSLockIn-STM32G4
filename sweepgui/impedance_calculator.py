import numpy as np

class ImpedanceCalculator:
    def __init__(self, r_ref, c_ref):
        self.r_ref = r_ref
        self.c_ref = c_ref
    
    def calculate_impedance_lsq(self, ch0, ch1, freq_hz, sample_rate):
        """
        Calculate impedance using least squares sine fit.
        ch0: voltage across DUT (V_dut)
        ch1: current through reference (I_ref) 
        freq_hz: known excitation frequency
        
        Returns: impedance magnitude, phase (degrees), and all fitted parameters
        """
        R_ref = self.r_ref * 1e3
        C_ref = self.c_ref * 1e-12  # Convert pF to F
        n_samples = len(ch0)
        
        # Time vector
        t = np.arange(n_samples) / sample_rate
        
        # Angular frequency
        omega = 2 * np.pi * freq_hz
        
        # Create design matrix for least squares fit
        # Model: A*cos(ωt) + B*sin(ωt) + C
        X = np.column_stack([
            np.cos(omega * t),
            np.sin(omega * t),
            np.ones_like(t)
        ])
        
        # Fit ch0 (V_dut - voltage across DUT)
        coeffs_ch0, residuals_ch0, *_ = np.linalg.lstsq(X, ch0, rcond=None)
        A_v, B_v, C_v = coeffs_ch0
        
        # Fit ch1 (I_ref - current through reference)
        coeffs_ch1, residuals_ch1, *_ = np.linalg.lstsq(X, ch1, rcond=None)
        A_i, B_i, C_i = coeffs_ch1
        
        # Convert to complex amplitude
        # Signal = A*cos(ωt) + B*sin(ωt) = Re{(A - jB)e^{jωt}}
        V_complex = A_v - 1j * B_v
        I_complex = A_i - 1j * B_i
        
        # Calculate phase of voltage channel for time alignment
        voltage_phase = np.angle(I_complex)  # in radians
        
        # Calculate impedance of parallel RC reference
        # Z_ref = (R_ref || C_ref) = 1/(1/R_ref + jωC_ref)
        # Or equivalently: Z_ref = R_ref / (1 + jωR_refC_ref)
        Z_ref_complex = R_ref / (1 + 1j * omega * R_ref * C_ref)
        
        # Calculate DUT impedance: Z_dut = V_dut / I_ref
        if abs(I_complex) > 1e-10:  # Avoid division by zero
            Z_dut_complex = I_complex / V_complex * Z_ref_complex
            Z_mag = abs(Z_dut_complex)
            Z_phase = np.angle(-Z_dut_complex, deg=True)
        else:
            Z_mag = 0
            Z_phase = 0
        
        # Alternative simple amplitude ratio (fallback)
        if Z_mag == 0 or not np.isfinite(Z_mag):
            # Use amplitude ratio method
            amp_v = np.sqrt(A_v**2 + B_v**2)
            amp_i = np.sqrt(A_i**2 + B_i**2)
            if amp_i > 0:
                Z_mag = amp_v / amp_i
                # Approximate phase from phase difference
                phase_v = np.arctan2(-B_v, A_v)  # Note: -B because of our convention
                phase_i = np.arctan2(-B_i, A_i)
                Z_phase = np.degrees(phase_v - phase_i)
                voltage_phase = phase_v  # Use approximate phase
            else:
                Z_mag = 0
                Z_phase = 0
                voltage_phase = 0
        
        # Normalize phase to [-180, 180]
        while Z_phase > 180:
            Z_phase -= 360
        while Z_phase < -180:
            Z_phase += 360
        
        # Calculate RMS of residuals
        if len(residuals_ch0) > 0:
            rms_residuals_ch0 = np.sqrt(residuals_ch0[0] / n_samples)
        else:
            rms_residuals_ch0 = 0
            
        if len(residuals_ch1) > 0:
            rms_residuals_ch1 = np.sqrt(residuals_ch1[0] / n_samples)
        else:
            rms_residuals_ch1 = 0
        
        return (Z_mag, Z_phase, voltage_phase, rms_residuals_ch0, rms_residuals_ch1, 
                C_v, C_i, A_v, B_v, A_i, B_i, V_complex, I_complex)