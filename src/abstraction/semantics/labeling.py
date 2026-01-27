from dataclasses import dataclass
from typing import List, Set

class Labels:
    CRASH = "crash"
    SAFE = "safe"
    BRAKING = "braking"      # AEB Active (Warn Level)
    EMERGENCY = "emergency"  # Panic Braking Active

class LabelingGrammar:
    """
    Defines the Semantic Mapping L(s).
    Maps Physical States (Gap, Velocity) -> Semantic Labels.
    Now implements DUAL-TRIGGER LOGIC (Distance OR TTC).
    """
    def __init__(self, controller):
        # 1. Extract Distance Thresholds
        self.dist_warn = getattr(controller, 'dist_warn', 15.0)
        self.dist_crit = getattr(controller, 'dist_emergency', 5.0)
        
        # 2. Extract TTC Thresholds (The missing link!)
        self.ttc_warn = getattr(controller, 'ttc_warn', 2.6) # Default Industry standard
        self.ttc_crit = getattr(controller, 'ttc_emergency', 1.6)

    def get_labels(self, continuous_state) -> List[str]:
        """
        Evaluates s |= alpha.
        Args:
            continuous_state: Tuple (Gap, V_rel, Accel)
        """
        gap = continuous_state[0]
        v_rel = continuous_state[1] # We now use Velocity!
        
        satisfied_props = set()

        # --- Rule 1: CRASH (Physics) ---
        if gap <= 0.0:
            satisfied_props.add(Labels.CRASH)
            return list(satisfied_props) # Terminal state
        else:
            satisfied_props.add(Labels.SAFE)

        # --- Rule 2: CALCULATE TTC ---
        # TTC is infinite if we are opening the gap (v_rel < 0)
        if v_rel > 0.1:
            ttc = gap / v_rel
        else:
            ttc = 999.0

        # --- Rule 3: DUAL-TRIGGER LOGIC (The Fix) ---
        
        # Condition A: Standard Braking
        # Triggered if Gap is too small OR TTC is too low
        if (gap < self.dist_warn) or (ttc < self.ttc_warn):
            satisfied_props.add(Labels.BRAKING)

        # Condition B: Emergency Braking
        # Triggered if Gap is critical OR TTC is critical
        if (gap < self.dist_crit) or (ttc < self.ttc_crit):
            satisfied_props.add(Labels.EMERGENCY)

        return list(satisfied_props)

    def __repr__(self):
        return (
            f"<Grammar | "
            f"Warn: (Dist<{self.dist_warn}m | TTC<{self.ttc_warn}s), "
            f"Crit: (Dist<{self.dist_crit}m | TTC<{self.ttc_crit}s)>"
        )