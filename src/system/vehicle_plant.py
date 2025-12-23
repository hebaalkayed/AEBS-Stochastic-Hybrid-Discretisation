import numpy as np

class BenchmarkAV:
    """
    Pure Physics Engine.
    Calculates the interaction between Ego Vehicle and Lead Vehicle.
    """
    def __init__(self, dt=0.2):
        self.dt = dt 
        
        # Physical Limits (Ego Vehicle)
        self.MAX_BRAKE = -9.8  # 1g
        self.MAX_ACCEL = 2.0   

    def get_next_state(self, x, u_ego, u_lead):
        """
        Inputs:
          x: State [Distance, V_ego, V_lead]
          u_ego: Acceleration of OUR car (from Controller)
          u_lead: Acceleration of THEIR car (from Environment)
        """
        dist, v_ego, v_lead = x
        
        # 1. Clamp Ego Input (Physical Limits)
        u_ego_actual = np.clip(u_ego, self.MAX_BRAKE, self.MAX_ACCEL)
        
        # 2. Update Velocities (Euler Integration)
        v_ego_next = v_ego + (u_ego_actual * self.dt)
        v_lead_next = v_lead + (u_lead * self.dt)
        
        # 3. Constraint: No reversing
        v_ego_next = max(0.0, v_ego_next)
        v_lead_next = max(0.0, v_lead_next)
        
        # 4. Update Distance
        # Distance changes by the difference in average speeds
        avg_v_ego = (v_ego + v_ego_next) / 2.0
        avg_v_lead = (v_lead + v_lead_next) / 2.0
        
        dist_next = dist + (avg_v_lead - avg_v_ego) * self.dt
        
        return np.array([dist_next, v_ego_next, v_lead_next])