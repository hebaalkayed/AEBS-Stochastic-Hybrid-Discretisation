import numpy as np

class AEBSController:
    """
    Reactive Controller.
    Agnostic to scenario type. Reacts purely based on Observed Gap and Closing Speed.
    """
    def __init__(self, dt=0.1, belief_noise_std=0.0, mode='safe', lead_behavior='static'):
        self.dt = dt
        self.state = 'drive'
        self.belief_noise = belief_noise_std
        self.est_velocity = 0.0 
        
        self.mode = mode               # 'industry' or 'safe'
        self.lead_behavior = lead_behavior 
        
        # --- BENCHMARK PARAMETERS ---
        if self.mode == 'industry':
            self.ttc_warn, self.ttc_brake, self.ttc_emergency = 2.6, 1.6, 1.0 
            self.dist_warn, self.dist_brake, self.dist_emergency = 10.0, 6.0, 2.0
            self.acc_brake, self.acc_emergency = -4.0, -9.8 
        elif self.mode == 'safe':
            self.ttc_warn, self.ttc_brake, self.ttc_emergency = 6.0, 5.0, 4.0 
            self.dist_warn, self.dist_brake, self.dist_emergency = 15.0, 10.0, 5.0
            self.acc_brake, self.acc_emergency = -4.0, -8.0 
            
        self.acc_coast = 0.0
        self.acc_drive = 1.0

    # --- SIMULATION INTERFACE ---
    def update_belief(self, requested_acc):
        prediction = self.est_velocity + (requested_acc * self.dt)
        noise = np.random.normal(0, self.belief_noise) if self.belief_noise > 0 else 0.0
        self.est_velocity = max(0.0, prediction + noise)

    def get_action(self, is_detected, obs_gap, obs_v_rel):
        """
        The Reactive Step.
        """
        # 1. Update Ego State (Belief)
        if self.est_velocity == 0.0 and obs_v_rel > 0: pass 

        # Default: Drive/Coast
        if not is_detected:
            self.state = 'drive'
            return self.acc_coast, 'coast'

        # 2. Calculate Risk Metrics
        if obs_v_rel > 0.1: # We are closing in
            ttc = obs_gap / obs_v_rel
        else:
            ttc = 999.0 

        req_acc = self.acc_coast
        self.state = 'coast'

        # 3. Decision Logic
        if obs_v_rel < 5.0:
            if obs_gap < self.dist_emergency: req_acc, self.state = self.acc_emergency, 'emergency_brake'
            elif obs_gap < self.dist_brake:   req_acc, self.state = self.acc_brake, 'brake'
        else:
            if ttc < self.ttc_emergency:      req_acc, self.state = self.acc_emergency, 'emergency_brake'
            elif ttc < self.ttc_brake:        req_acc, self.state = self.acc_brake, 'brake'

        self.update_belief(req_acc)
        return req_acc, self.state

    # --- VERIFICATION INTERFACE ---
    def get_action_name_for_state(self, x, v, a, plant_coords='relative_frame'):
        """
        Determines action for a specific state in the Abstraction Grid.
        Args:
            x: State Variable 1 (Position or Gap)
            plant_coords: 'relative_frame' (x=Gap) or 'world_frame' (x=Position)
        """
        # 1. Normalize Inputs
        if plant_coords == 'relative_frame':
            sim_gap = x
            sim_v_rel = v
        else:
            # Assume 100m track for world frame verification
            sim_gap = 100.0 - x
            sim_v_rel = v 

        if sim_gap < 0: sim_gap = 0.0

        # 2. Call the Reactive Logic
        _, action_name = self.get_action(True, sim_gap, sim_v_rel)
        
        # Map back to simple names for PRISM
        if action_name == 'emergency_brake': return 'brake_full'
        if action_name == 'brake': return 'brake_warn'
        return 'coast'

    def generate_prism_logic(self, grid, plant_coords='relative_frame') -> str:
        """Generates PRISM formulas."""
        action_states = {'brake_full': [], 'brake_warn': [], 'coast': []}
        
        import itertools
        ranges = [range(d) for d in grid.shape]
        
        for idx_tuple in itertools.product(*ranges):
            flat_id = grid.get_flat_index(idx_tuple)
            center = grid.index_to_cell_center(idx_tuple)
            gap, v_rel, _ = center 
            
            action_name = self.get_action_name_for_state(gap, v_rel, 0, plant_coords=plant_coords)
            
            if action_name in action_states:
                action_states[action_name].append(str(flat_id))
                
        lines = [f"\n// --- Controller Logic: {self.mode.upper()} ---"]
        for action, states in action_states.items():
            condition = "(" + " | ".join([f"s={s}" for s in states]) + ")" if states else "false"
            lines.append(f"formula do_{action} = {condition};")
        
        lines.append(f"\nmodule Controller_{self.mode}")
        for action in action_states.keys():
            lines.append(f"    [{action}] do_{action} -> true;")
        lines.append("endmodule")
        return "\n".join(lines)