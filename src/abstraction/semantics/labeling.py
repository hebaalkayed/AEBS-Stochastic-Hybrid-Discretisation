class Labels:
    CRASH = "crash"
    SAFE = "safe"
    BRAKING = "braking"
    EMERGENCY = "emergency"

class LabelingGrammar:
    def __init__(self, controller):
        # Dynamic binding to controller thresholds
        self.dist_warn = getattr(controller, 'dist_warn', 15.0)
        self.dist_crit = getattr(controller, 'dist_emergency', 5.0)
        self.ttc_warn = getattr(controller, 'ttc_warn', 2.6)
        self.ttc_crit = getattr(controller, 'ttc_emergency', 1.6)

    def get_labels(self, state):
        """
        Returns list of labels satisfied by state (Gap, V_ego, V_lead).
        Implements Dual-Trigger Logic (Distance OR TTC).
        """
        # BUG FIX: Use closing speed v_rel = v_ego - v_lead, NOT raw v_ego.
        gap, v_ego, v_lead = state[0], state[1], state[2]
        v_rel = v_ego - v_lead
        
        props = set()
        
        if gap <= 0:
            return [Labels.CRASH]
        props.add(Labels.SAFE)
        
        # Calculate TTC using closing speed (avoid divide by zero)
        ttc = gap / v_rel if v_rel > 0.1 else 999.0
        
        # Check Safety Rules
        if (gap < self.dist_warn) or (ttc < self.ttc_warn):
            props.add(Labels.BRAKING)
            
        if (gap < self.dist_crit) or (ttc < self.ttc_crit):
            props.add(Labels.EMERGENCY)
        
        return list(props)