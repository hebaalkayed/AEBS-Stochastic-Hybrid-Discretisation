import numpy as np
from src.abstraction.types.interfaces import StochasticHybridSystem

class PlantWrapper(StochasticHybridSystem):
    def __init__(self, plant):
        self.plant = plant
    
    def get_action_space(self):
        # Maps Discrete ID -> Continuous Accel
        return {0: 0.0, 1: -4.0, 2: -9.8} # Coast, Brake, Emergency

    def get_next_state_distribution(self, s, a):
        return (
            self.plant.get_deterministic_next_state(s, a),
            self.plant.noise_std,
            self.plant.lipschitz_constant
        )

class ControllerWrapper(StochasticHybridSystem):
    def __init__(self, controller):
        self.ctrl = controller
        self.map = {'coast': 0, 'brake_warn': 1, 'brake_full': 2}

    def get_action_space(self):
        return {0: 0.0} # Dummy input for controller

    def get_next_state_distribution(self, s, a):
        # s is Observation (Gap, V), return value is Action ID
        gap, v = s[0], s[1]
        
        # We query the controller logic
        act_name = self.ctrl.get_action_name_for_state(gap, v, 0, plant_coords='relative_frame')
        act_id = self.map.get(act_name, 0)
        
        # Return deterministic transition to the Action ID
        return np.array([act_id, 0, 0]), 0.0, 0.0