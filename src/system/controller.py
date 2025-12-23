import numpy as np

class AEBSController:
    """
    AEBS Controller with Dual Modes.
    - High Speed: Uses TTC (Time-To-Collision).
    - Low Speed: Uses Distance Thresholds (to ensure full stop).
    """
    def __init__(self, dt=0.2, belief_noise_std=0.0, mode='safe'):
        self.dt = dt
        self.state = 'drive'
        self.belief_noise = belief_noise_std
        self.est_velocity = 0.0 
        self.mode = mode
        
        # --- PARAMETERS ---
        if self.mode == 'industry':
            # AGGRESSIVE (Late Braking)
            # TTC Thresholds (High Speed)
            self.ttc_warn      = 2.6
            self.ttc_brake     = 1.6
            self.ttc_emergency = 1.0 
            
            # Distance Thresholds (Low Speed < 5m/s)
            self.dist_warn      = 10.0
            self.dist_brake     = 6.0
            self.dist_emergency = 2.0
            
            # Actions
            self.acc_brake     = -4.0
            self.acc_emergency = -9.8 
            
        elif self.mode == 'safe':
            # CONSERVATIVE (Early Braking)
            # TTC Thresholds (High Speed)
            self.ttc_warn      = 6.0
            self.ttc_brake     = 5.0
            self.ttc_emergency = 4.0 
            
            # Distance Thresholds (Low Speed < 5m/s)
            self.dist_warn      = 15.0
            self.dist_brake     = 10.0
            self.dist_emergency = 5.0
            
            # Actions
            self.acc_brake     = -4.0
            self.acc_emergency = -8.0 
            
        self.acc_coast = 0.0
        self.acc_drive = 1.0

    def update_belief(self, requested_acc):
        prediction = self.est_velocity + (requested_acc * self.dt)
        if self.belief_noise > 0:
            noise = np.random.normal(0, self.belief_noise)
        else:
            noise = 0.0
        self.est_velocity = max(0.0, prediction + noise)

    def get_action(self, perception_output, observed_distance):
        # 1. Sync initial belief if needed
        if self.est_velocity == 0.0: pass

        req_acc = self.acc_drive
        self.state = 'drive'

        # 2. STOPPED CHECK (Only if truly stopped)
        if self.est_velocity < 0.1:
            req_acc = 0.0
            self.state = 'stopped'
            self.update_belief(req_acc)
            return req_acc

        # 3. HYBRID LOGIC
        if perception_output == 1: # Obstacle Detected
            
            # A. LOW SPEED LOGIC (Use Distance)
            # TTC is unstable at low speeds, so we use fixed distances.
            if self.est_velocity < 5.0:
                if observed_distance < self.dist_emergency:
                    self.state = 'emergency_brake'
                    req_acc = self.acc_emergency
                elif observed_distance < self.dist_brake:
                    self.state = 'brake'
                    req_acc = self.acc_brake
                elif observed_distance < self.dist_warn:
                    self.state = 'coast'
                    req_acc = self.acc_coast
            
            # B. HIGH SPEED LOGIC (Use TTC)
            else:
                ttc = observed_distance / self.est_velocity
                if ttc < self.ttc_emergency:
                    self.state = 'emergency_brake'
                    req_acc = self.acc_emergency
                elif ttc < self.ttc_brake:
                    self.state = 'brake'
                    req_acc = self.acc_brake
                elif ttc < self.ttc_warn:
                    self.state = 'coast'
                    req_acc = self.acc_coast

        # 4. Update Belief
        self.update_belief(req_acc)
        return req_acc