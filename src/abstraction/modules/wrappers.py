import numpy as np
from src.abstraction.types.interfaces import StochasticHybridSystem

class PlantWrapper(StochasticHybridSystem):
    def __init__(self, plant, controller, lead_noise_std=2.0):
        """
        Args:
            lead_noise_std: Standard deviation of lead vehicle acceleration (m/s^2).
        """
        self.plant = plant
        self.ctrl = controller
        
        # Combine Plant Noise (Engine) and Lead Noise (Driver)
        # This is strictly in ACCELERATION units (m/s^2)
        self.accel_sigma = np.sqrt(self.plant.noise_std**2 + lead_noise_std**2)

    def get_action_space(self):
        return {
            0: self.ctrl.acc_coast,      # 0.0
            1: self.ctrl.acc_brake,      # -4.0
            2: self.ctrl.acc_emergency   # -9.8 (Safe) or -8.0 (Industry)
        }

    def get_next_state_distribution(self, s, a):
        # 1. Deterministic Physics Step
        next_state_det = self.plant.get_deterministic_next_state(s, a)
        
        # 2. CRITICAL FIX: Convert Acceleration Noise to Velocity Noise
        # Physics: Delta_V = Acceleration * Time
        # Therefore: Sigma_V = Sigma_A * dt
        dt = self.plant.dt
        sigma_on_velocity = self.accel_sigma * dt
        
        return (
            next_state_det,
            sigma_on_velocity,
            self.plant.lipschitz_constant
        )

class ControllerWrapper(StochasticHybridSystem):
    # NOTE: This wrapper is no longer used by the new Pipeline to prevent the 
    # "7482" coordinate mapping bug. The pipeline now generates the controller 
    # lookup table directly. We keep this class here only to avoid breaking legacy imports.
    def __init__(self, controller):
        self.ctrl = controller
        self.map = {'coast': 0, 'brake_warn': 1, 'brake_full': 2}
    def get_action_space(self): return {0: 0.0}
    def get_next_state_distribution(self, s, a): return np.array([0,0,0]), 0.0, 0.0