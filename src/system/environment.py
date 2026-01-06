import numpy as np

class TrafficEnvironment:
    """
    The Source of Truth.
    Manages the 'Real World' dynamics of the Lead Vehicle based on User Configuration.
    """
    def __init__(self, dt=0.1):
        self.dt = dt
        
        # Scenario Configuration (Set by User/CMD)
        self.lead_type = 'static' 
        
        # Physical State (Ground Truth)
        self.gap = 0.0
        self.ego_velocity = 0.0
        self.lead_velocity = 0.0
        self.lead_acceleration = 0.0
        
        # --- STOCHASTIC KNOBS (Tweaked for Chaos) ---
        # 1. Base Noise: Standard "wobbly" driving
        self.noise_std = 2.0  
        
        # 2. Jerk Probability: Chance of a sudden event per time step
        # If dt=0.1, 0.05 means roughly one "event" every 2 seconds.
        self.jerk_probability = 0.05 

    def configure(self, scenario_type, initial_gap, initial_ego_v, initial_lead_v):
        """
        INJECTION POINT: Initializing the scenario parameters from the CMD/User.
        """
        self.lead_type = scenario_type
        self.gap = float(initial_gap)
        self.ego_velocity = float(initial_ego_v)
        self.lead_velocity = float(initial_lead_v)
        self.lead_acceleration = 0.0
        
        print(f"[Environment] Configured: Type={self.lead_type}, Gap={self.gap}m, LeadV={self.lead_velocity}m/s")

    def update_physics(self, ego_displacement, ego_velocity):
        """
        Advances the world one time step.
        """
        # A. Determine Lead Dynamics
        if self.lead_type == 'static':
            self.lead_acceleration = 0.0
            self.lead_velocity = 0.0
            
        elif self.lead_type == 'steady':
            self.lead_acceleration = 0.0
            # lead_velocity stays constant
            
        elif self.lead_type == 'unpredictable':
            # 1. Base Noise (Gaussian Walk) - The car drifts in speed
            base_accel = np.random.normal(0, self.noise_std)
            
            # 2. "Event" Logic (The Chaos)
            # We roll a die to see if the driver does something crazy
            if np.random.rand() < self.jerk_probability:
                # 50/50 chance of Hard Brake vs Hard Acceleration
                if np.random.rand() < 0.5:
                    base_accel = -8.0 # HARD BRAKE CHECK!
                else:
                    base_accel = 5.0  # TRY TO RUN AWAY
            
            self.lead_acceleration = base_accel
            self.lead_velocity += self.lead_acceleration * self.dt
            
            # Physics Constraints (prevent reversing or going supersonic)
            if self.lead_velocity < 0: self.lead_velocity = 0.0
            if self.lead_velocity > 50: self.lead_velocity = 50.0

        # B. Calculate Lead Displacement
        lead_displacement = self.lead_velocity * self.dt
        
        # C. Update Gap (Gap shrinks if Ego > Lead)
        self.gap = self.gap - (ego_displacement - lead_displacement)
        
        # Update internal ego record for plotting/logging
        self.ego_velocity = ego_velocity
        
        return self.get_ground_truth()

    def get_ground_truth(self):
        """
        Returns the raw, perfect state of the world.
        """
        return {
            'gap': self.gap,
            'v_ego': self.ego_velocity,
            'v_lead': self.lead_velocity,
            'v_rel': self.ego_velocity - self.lead_velocity # Closing speed
        }