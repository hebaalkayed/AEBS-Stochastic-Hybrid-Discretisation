import numpy as np

class PerceptionSystem:
    """
    The Bridge between Environment and Controller.
    Simulates Sensor Noise and DNN False Negatives.
    """
    def __init__(self, false_negative_rate=0.0, position_noise_std=0.0, velocity_noise_std=0.0):
        self.fn_rate = false_negative_rate
        self.pos_noise = position_noise_std
        self.vel_noise = velocity_noise_std

    def read_sensors(self, ground_truth):
        """
        Takes Ground Truth -> Returns Observed State (inputs for Controller).
        Returns:
            - is_detected (bool): Did the DNN see the car?
            - obs_gap (float): Noisy distance reading
            - obs_v_rel (float): Noisy closing speed reading
        """
        real_gap = ground_truth['gap']
        real_v_rel = ground_truth['v_rel']
        
        # 1. Detection Logic (Classification DNN)
        # Check range (e.g. max sensor range 100m)
        if real_gap > 100.0:
            return False, 100.0, 0.0
            
        # Apply False Negative Rate
        if np.random.rand() < self.fn_rate:
            return False, real_gap, real_v_rel # Missed it!
            
        # 2. Estimation Logic (Regression DNN)
        # Add Gaussian noise to measurements
        gap_noise = np.random.normal(0, self.pos_noise)
        vel_noise = np.random.normal(0, self.vel_noise)
        
        obs_gap = max(0.0, real_gap + gap_noise)
        obs_v_rel = real_v_rel + vel_noise # Note: v_rel can be negative (opening gap)
        
        return True, obs_gap, obs_v_rel