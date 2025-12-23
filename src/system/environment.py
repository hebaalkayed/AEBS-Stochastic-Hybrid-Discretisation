import numpy as np

class TrafficEnvironment:
    """
    Manages Road Scenarios based on Euro NCAP and ISO Benchmarks.
    """
    def __init__(self, dt=0.2):
        self.dt = dt
        self.scenario = None
        self.ego_speed = 0.0
        self.lead_speed = 0.0
        self.gap = 0.0
        self.lead_behavior = 'static'
        
        self.time_elapsed = 0.0
        self.noise_std = 1.0

    def reset(self, scenario_type):
        """
        Sets initial conditions based on Industry Standards.
        Returns: initial_state_vector [Gap, V_ego, V_lead]
        """
        self.scenario = scenario_type
        self.time_elapsed = 0.0
        
        # --- SCENARIO 1: URBAN (AEB City) ---
        # Ref: Euro NCAP AEB City Protocol
        # Test Range: 10-50 km/h.
        # We use 36 km/h (10 m/s) as the standard urban baseline.
        if scenario_type == 'urban_static':
            self.ego_speed = 10.0  # 36 km/h
            self.lead_speed = 0.0
            self.gap = 40.0        # Standard visual range start
            self.lead_behavior = 'static'

        elif scenario_type == 'urban_cutout':
            # Simulates a 'Cut-Out' or late detection.
            # TTC = 1.5s (Reaction time boundary). 
            # Gap = 10 m/s * 1.5s = 15.0m
            self.ego_speed = 10.0
            self.lead_speed = 0.0
            self.gap = 15.0        
            self.lead_behavior = 'static'
            
        # --- SCENARIO 2: HIGHWAY (AEB Inter-Urban) ---
        # Ref: Euro NCAP AEB Inter-Urban Protocol
        # Stress Test Limit: 90 km/h (25 m/s).
        elif scenario_type == 'highway_static':
            self.ego_speed = 25.0  # 90 km/h
            self.lead_speed = 0.0
            self.gap = 120.0       # Long range radar detection
            self.lead_behavior = 'static'

        elif scenario_type == 'highway_cutout':
            # The "Impossible" Physics Test.
            # Stopping Dist @ 25m/s (-9.8m/s^2) is ~32m.
            # We set Gap to 40m (1.6s TTC) -> The absolute edge of safety.
            self.ego_speed = 25.0
            self.lead_speed = 0.0
            self.gap = 40.0        
            self.lead_behavior = 'static'
            
        elif scenario_type == 'highway_traffic':
            self.ego_speed = 25.0
            self.lead_speed = 25.0 
            self.gap = 40.0        
            self.lead_behavior = 'unpredictable'
            
        else:
            raise ValueError(f"Unknown scenario: {scenario_type}")

        return np.array([self.gap, self.ego_speed, self.lead_speed])

    def get_lead_action(self):
        u_lead = 0.0
        
        if self.lead_behavior == 'static':
            u_lead = 0.0
        elif self.lead_behavior == 'unpredictable':
            u_lead = np.random.normal(0, self.noise_std)
            
        self.lead_speed += u_lead * self.dt
        if self.lead_speed < 0: self.lead_speed = 0
        self.time_elapsed += self.dt
        
        return u_lead