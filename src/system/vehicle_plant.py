import math
import numpy as np

class VehiclePlant:
    """
    The "Real" Physics Plant.
    
    Coordinate Systems:
    - 'world_frame': x is position on track. v is velocity. (x increases with v)
      -> Used for Visual Simulation.
      
    - 'relative_frame': x is distance to obstacle (Gap). v is closing speed. (x decreases with v)
      -> Used for Formal Verification (PRISM).
    """
    def __init__(self, x=0.0, y=0.0, theta=0.0, v=0.0, a=0.0, alpha=0.5, dt=0.1, coordinate_system='world_frame'):
        self.x = x
        self.y = y
        self.theta = theta
        self.actual_velocity = v
        self.actual_acceleration = a
        
        # Dynamics Parameters
        self.alpha = alpha  # Lag factor
        self.dt = dt
        self.coordinate_system = coordinate_system 
        
        # Abstraction Parameters
        self.process_noise_std = 0.05 

    def step(self, action_acceleration):
        """
        Advances the system by one time step (dt).
        """
        # 1. Update Acceleration (First-Order Lag)
        self.actual_acceleration = self.actual_acceleration + self.alpha * (action_acceleration - self.actual_acceleration)

        # 2. Update Velocity (Or Closing Speed)
        self.actual_velocity += self.actual_acceleration * self.dt
        
        # Constraints: No negative velocity
        if self.actual_velocity < 0.0:
            self.actual_velocity = 0.0
            self.actual_acceleration = 0.0

        # 3. Update Position (Longitudinal)
        if self.coordinate_system == 'world_frame':
            # World Frame: x increases as we drive forward
            self.x += self.actual_velocity * self.dt
        else:
            # Relative Frame: x is GAP. Positive closing speed REDUCES the gap.
            self.x -= self.actual_velocity * self.dt

    # =========================================================
    #  ABSTRACTION INTERFACE
    # =========================================================

    def get_deterministic_next_state(self, state_vector, action_acceleration):
        x, v, a = state_vector
        
        # 1. Acceleration
        a_next = a + self.alpha * (action_acceleration - a)
        
        # 2. Velocity
        v_next = v + a_next * self.dt
        if v_next < 0:
            v_next = 0.0
            a_next = 0.0
            
        # 3. Position/Gap
        if self.coordinate_system == 'world_frame':
            x_next = x + (v_next * self.dt)
        else:
            x_next = x - (v_next * self.dt)
        
        return np.array([x_next, v_next, a_next])

    @property
    def lipschitz_constant(self):
        # Direction depends on frame: +1 for World, -1 for Relative
        direction = -1.0 if self.coordinate_system == 'relative_frame' else 1.0
        
        A = np.array([
            [1.0, direction * self.dt, direction * (1 - self.alpha) * (self.dt ** 2)],
            [0.0, 1.0,                 (1 - self.alpha) * self.dt],
            [0.0, 0.0,                 (1 - self.alpha)]
        ])
        return np.linalg.norm(A, ord=2)

    @property
    def noise_std(self):
        return self.process_noise_std