class POCController:
    """
    Micro-World AEBS Controller.
    
    A scaled-down version of AEBSController for the 16m POC road.
    Same dual-trigger logic (TTC for fast closing, distance for slow closing),
    with thresholds tuned so that all three actions appear across the grid.
    
    Production (industry):  ttc_emergency=1.0,  ttc_brake=1.6,  acc_emergency=-9.8
    POC:                    ttc_emergency=1.5,  ttc_brake=3.0,  acc_emergency=-4.0
    
    Key decisions (v_lead=0, so v_rel = v_ego):
      (gap=16, v=4): TTC=4.0 > 3.0        → coast
      (gap=12, v=4): TTC=3.0 <= 3.0       → brake
      (gap= 4, v=6): TTC=0.67 <= 1.5      → emergency
      (gap= 4, v=2): v_rel<2, gap<5       → brake (distance trigger)
      (gap= 2, v=2): v_rel<2, gap<=2      → emergency (distance trigger)
    """

    def __init__(self):
        self.dt = 0.1

        # Scaled accelerations (production: 0, -4, -9.8)
        self.acc_coast = 0.0
        self.acc_brake = -2.0
        self.acc_emergency = -4.0

        # Distance thresholds for slow closing (v_rel < 2 m/s)
        self.dist_warn = 8.0
        self.dist_brake = 5.0
        self.dist_emergency = 2.0

        # TTC thresholds for fast closing (v_rel >= 2 m/s)
        self.ttc_warn = 3.0
        self.ttc_brake = 3.0       # same as warn (used by labeling grammar)
        self.ttc_emergency = 1.5

    def get_action(self, is_detected, obs_gap, obs_v_rel):
        """
        Dual-trigger reactive controller.
        obs_v_rel = closing speed = v_ego - v_lead (positive means approaching).
        """
        if not is_detected:
            return self.acc_coast, 'coast'

        ttc = obs_gap / obs_v_rel if obs_v_rel > 0.1 else 999.0

        # Slow closing: use distance thresholds
        if obs_v_rel < 2.0:
            if obs_gap <= self.dist_emergency:
                return self.acc_emergency, 'emergency_brake'
            if obs_gap <= self.dist_brake:
                return self.acc_brake, 'brake'
        # Fast closing: use TTC thresholds
        else:
            if ttc <= self.ttc_emergency:
                return self.acc_emergency, 'emergency_brake'
            if ttc <= self.ttc_warn:
                return self.acc_brake, 'brake'

        return self.acc_coast, 'coast'

    def get_action_name_for_state(self, x, v, a, plant_coords='relative_frame'):
        """
        Maps a state to an action name for the abstraction.
        x = gap, v = closing speed (v_rel = v_ego - v_lead).
        """
        sim_gap = max(0.0, x)
        _, action_name = self.get_action(True, sim_gap, v)

        if action_name == 'emergency_brake':
            return 'brake_full'
        if action_name == 'brake':
            return 'brake_warn'
        return 'coast'