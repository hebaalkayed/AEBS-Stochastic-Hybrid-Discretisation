import math
import numpy as np

class VehiclePlant:
    """
    The "Real" Physics Plant.
    
    Dynamics Sources:
    - Discrete update logic matches ARCH-COMP20 AINNCS Benchmark (Lag dynamics).
    - Parameters: alpha=0.5 (Responsiveness), dt=0.1s (Step size).
    """
    def __init__(self, x=0.0, y=0.0, theta=0.0, v=0.0, a=0.0, alpha=0.5, dt=0.1):
        self.x = x
        self.y = y
        self.theta = theta
        self.actual_velocity = v
        self.actual_acceleration = a
        
        # Dynamics Parameters
        self.alpha = alpha  # Lag factor
        self.dt = dt
        
        # Abstraction Parameters (Noise for the Stochastic Kernel)
        # Assuming additive Gaussian noise on acceleration/velocity for the abstraction
        self.process_noise_std = 0.05 

    def step(self, action_acceleration):
        """
        Advances the system by one time step (dt) with the given control input.
        This updates the internal state of the vehicle.
        """
        # 1. Update Acceleration (First-Order Lag)
        # a_next = a + alpha * (u - a)
        self.actual_acceleration = self.actual_acceleration + self.alpha * (action_acceleration - self.actual_acceleration)

        # 2. Update Velocity
        # v_next = v + a_next * dt
        self.actual_velocity += self.actual_acceleration * self.dt
        
        # Constraints: No negative velocity
        if self.actual_velocity < 0.0:
            self.actual_velocity = 0.0
            self.actual_acceleration = 0.0

        # 3. Update Position (Longitudinal)
        # x_next = x + v_next * cos(theta) * dt
        self.x += self.actual_velocity * math.cos(self.theta) * self.dt
        self.y += self.actual_velocity * math.sin(self.theta) * self.dt

    # =========================================================
    #  ABSTRACTION INTERFACE (For Algorithm 1 in Survey)
    # =========================================================

    def get_deterministic_next_state(self, state_vector, action_acceleration):
        """
        Calculates f(x, u) - The exact next state if there were NO noise.
        
        Args:
            state_vector: np.array([x, v, a])
            action_acceleration: float (control input 'u')
            
        Returns:
            np.array([x_next, v_next, a_next])
        """
        x, v, a = state_vector
        
        # Replicate the exact math from step(), but stateless and noiseless
        
        # 1. Acceleration Dynamics
        a_next = a + self.alpha * (action_acceleration - a)
        
        # 2. Velocity Dynamics
        v_next = v + a_next * self.dt
        if v_next < 0:
            v_next = 0.0
            a_next = 0.0
            
        # 3. Position Dynamics (Assuming 1D/Linear for the Grid Abstraction)
        x_next = x + v_next * self.dt
        
        return np.array([x_next, v_next, a_next])

    @property
    def lipschitz_constant(self):
        """
        Returns L: The Lipschitz constant of the discrete-time dynamics.
        Used to bound the error of the abstraction (Algorithm 1, Line 2).
        
        Calculated as the spectral norm (max singular value) of the A matrix
        representing the linear update: s_{k+1} = A * s_k + B * u_k
        """
        # Construct the A matrix for the update equations:
        # a' = (1-alpha)a
        # v' = v + a'dt = v + (1-alpha)dt*a
        # x' = x + v'dt = x + dt*v + (1-alpha)dt^2*a
        
        A = np.array([
            [1.0, self.dt, (1 - self.alpha) * (self.dt ** 2)],  # x row
            [0.0, 1.0,     (1 - self.alpha) * self.dt],         # v row
            [0.0, 0.0,     (1 - self.alpha)]                    # a row
        ])
        
        # Return spectral norm (largest singular value)
        return np.linalg.norm(A, ord=2)

    @property
    def noise_std(self):
        """Exposes the process noise standard deviation for integration."""
        return self.process_noise_std