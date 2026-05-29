import numpy as np


class POCVehiclePlant:
    """
    Micro-World Vehicle Plant.
    
    A scaled-down version of VehiclePlant that operates on a 16m road
    at walking/cycling speeds. Same dynamics, smaller numbers.
    
    Production system: 100m road, v up to 30 m/s, dt=0.1s, alpha=0.5
    POC system:         16m road, v up to  6 m/s, dt=1.0s, alpha=1.0
    
    Key invariant: all three actions (coast/brake/emergency) land in
    DIFFERENT grid cells, making the IMDP meaningful for verification.
    
    Stopping distances (from v=4 m/s, v_lead=0):
      emergency (a=-4): v→0 in 1 step,  gap consumed = 0m  (instant stop)
      brake     (a=-2): v→2→0 in 2 steps, gap consumed = 2m
      coast     (a= 0): never stops,      gap consumed = 4m/step
    """

    def __init__(self, dt=1.0, alpha=1.0):
        self.dt = 0.1      # was 1.0 — 10x more steps to stop
        self.alpha = 0.5    # was 1.0 — braking force halved

        # High relative noise (0.8 m/s² vs production's 0.05 m/s²).
        # Combined with lead_noise_std=0.5 in the wrapper, this gives
        # sigma_on_velocity ≈ 0.94 m/s — a good ratio to 1 m/s v_lead cells.
        self.process_noise_std = 0.8

    def get_deterministic_next_state(self, state_vector, action_acceleration):
        """
        Same 3D AEBS dynamics as VehiclePlant:
          State: (gap, v_ego, v_lead)
          
          a_ego      = alpha * action
          v_ego'     = max(0, v_ego + a_ego * dt)
          v_lead'    = max(0, v_lead)       (constant, deterministic)
          gap'       = gap + (v_lead' - v_ego') * dt
        """
        gap, v_ego, v_lead = state_vector

        a_ego = self.alpha * action_acceleration
        v_ego_next = max(0.0, v_ego + a_ego * self.dt)
        v_lead_next = max(0.0, v_lead)
        gap_next = gap + (v_lead_next - v_ego_next) * self.dt

        return np.array([gap_next, v_ego_next, v_lead_next])

    @property
    def noise_std(self):
        return self.process_noise_std