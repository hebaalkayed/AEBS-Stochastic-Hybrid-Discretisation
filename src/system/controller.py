import numpy as np

class AEBSController:
    """
    AEBS Controller with Dual Modes.
    - High Speed: Uses TTC (Time-To-Collision).
    - Low Speed: Uses Distance Thresholds (to ensure full stop).
    """
    def __init__(self, dt=0.1, belief_noise_std=0.0, mode='safe'):
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

    # =========================================================================
    #  PART 1: SIMULATION INTERFACE
    #  Used by your Python Simulator (step-by-step execution)
    # =========================================================================

    def update_belief(self, requested_acc):
        prediction = self.est_velocity + (requested_acc * self.dt)
        if self.belief_noise > 0:
            noise = np.random.normal(0, self.belief_noise)
        else:
            noise = 0.0
        self.est_velocity = max(0.0, prediction + noise)

    def get_action(self, perception_output, observed_distance):
        """
        Calculates the control action based on current inputs.
        Args:
            perception_output (int): 1 if obstacle detected, 0 otherwise.
            observed_distance (float): Distance to obstacle in meters.
        """
        # 1. Sync initial belief if needed (first step)
        if self.est_velocity == 0.0: pass # Real logic would init from sensor

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
            # TTC is unstable at low speeds (divide by zero risk), so we use fixed distances.
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
                else:
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
                else:
                    self.state = 'coast'
                    req_acc = self.acc_coast

        # 4. Update Belief
        self.update_belief(req_acc)
        return req_acc

    # =========================================================================
    #  PART 2: FORMAL VERIFICATION INTERFACE
    #  Used by the Abstraction Pipeline to generate PRISM code.
    # =========================================================================

    def get_action_name_for_state(self, x, v, a):
        """
        Runs the logic 'dry' on a specific state (x,v,a) to see what 
        the controller WOULD do. 
        
        Assumptions for Verification:
        1. x represents 'observed_distance' (Distance to Obstacle).
        2. v represents 'est_velocity'.
        3. perception_output is ALWAYS 1 (We verify the collision scenario).
        """
        # Map Grid State to Controller Inputs
        sim_v = v
        sim_dist = x
        
        # STOPPED CHECK
        if sim_v < 0.1:
            return 'coast' # Mapping stopped behavior to coast for the MDP dynamics

        # LOW SPEED LOGIC (< 5.0 m/s)
        if sim_v < 5.0:
            if sim_dist < self.dist_emergency: return 'brake_full'
            if sim_dist < self.dist_brake:     return 'brake_warn'
            return 'coast' # Covers warn and safe zones
            
        # HIGH SPEED LOGIC (>= 5.0 m/s)
        else:
            ttc = sim_dist / sim_v if sim_v > 0 else 999.0
            if ttc < self.ttc_emergency:   return 'brake_full'
            if ttc < self.ttc_brake:       return 'brake_warn'
            return 'coast'

    def generate_prism_logic(self, grid) -> str:
        """
        Scans the Grid, applies the logic above, and generates
        PRISM formulas grouping states by Action.
        """
        print(f"    [Controller] Generating PRISM logic for mode: {self.mode}")
        
        # Buckets to hold state IDs for each action
        # Keys must match the action names used in the Plant MDP
        action_states = {
            'brake_full': [],
            'brake_warn': [],
            'coast': []
        }
        
        # Loop over every cell in the grid using the Grid object's own iterator logic
        import itertools
        ranges = [range(d) for d in grid.shape]
        
        for idx_tuple in itertools.product(*ranges):
            flat_id = grid.get_flat_index(idx_tuple)
            center = grid.index_to_cell_center(idx_tuple)
            x, v, a = center
            
            # Ask the controller what it wants to do here
            action_name = self.get_action_name_for_state(x, v, a)
            
            if action_name in action_states:
                action_states[action_name].append(str(flat_id))
            else:
                # Fallback for unmapped actions (should not happen if consistent)
                pass
                
        # Generate Text Block
        lines = []
        lines.append(f"\n// --- Controller Logic: {self.mode.upper()} ---")
        lines.append("// Auto-generated by src.system.controller.py")
        
        # Create Formulas: formula do_brake = (s=1 | s=2 ...);
        for action, states in action_states.items():
            if not states:
                condition = "false"
            else:
                # Join states with OR logic
                condition = "(" + " | ".join([f"s={s}" for s in states]) + ")"
            
            lines.append(f"formula do_{action} = {condition};")
            
        # Create The Module
        lines.append(f"\nmodule Controller_{self.mode}")
        lines.append(f"    // Deterministic Transitions based on formulas")
        
        # Logic: If the formula for an action is true, allow that action
        for action in action_states.keys():
            lines.append(f"    [{action}] do_{action} -> true;")
            
        lines.append("endmodule")
        
        return "\n".join(lines)