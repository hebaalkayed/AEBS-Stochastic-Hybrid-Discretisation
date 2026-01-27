from dataclasses import dataclass, field
from typing import Dict, Tuple, List, Optional
from collections import defaultdict
import os

# Type Aliases
StateID = int
ActionName = str

@dataclass
class Transition:
    """
    Represents a probabilistic jump to a target state.
    For IMDPs (Interval MDPs), we store the probability bounds.
    """
    target_id: StateID
    p_min: float
    p_max: float

    @property
    def p_mean(self) -> float:
        """Returns the average probability (used for standard MDP export)."""
        return (self.p_min + self.p_max) / 2.0

@dataclass
class MDPState:
    """Optional metadata for a state (e.g., continuous center, safety label)."""
    id: StateID
    continuous_center: Tuple[float, ...]
    labels: List[str] = field(default_factory=list)

    @property
    def is_crash(self) -> bool:
        return "crash" in self.labels

@dataclass
class MDP:
    """
    The mathematical artifact produced by Algorithm 1.
    Represents a Finite Interval Markov Decision Process.
    """
    num_states: int
    
    # 1. Transitions: (source_id, action) -> List of Transitions
    transitions: Dict[Tuple[StateID, ActionName], List[Transition]] = field(default_factory=dict)
    
    # 2. State Metadata: ID -> Struct (For Python Debugging/Inspection)
    states: Dict[StateID, MDPState] = field(default_factory=dict)
    
    # 3. Label Groups: LabelName -> List of IDs (For PRISM Export)
    labels: Dict[str, List[int]] = field(default_factory=lambda: defaultdict(list))
    
    # 4. Controller Logic String
    controller_logic: str = ""

    def add_transition(self, src: StateID, action: ActionName, target: StateID, p_min: float, p_max: float):
        key = (src, action)
        if key not in self.transitions:
            self.transitions[key] = []
        self.transitions[key].append(Transition(target, p_min, p_max))

    def add_state_info(self, state_id: StateID, center: Tuple[float, ...], state_labels: List[str]):
        """
        Populates BOTH the debugging struct and the PRISM label groups.
        """
        # A. Store Debug Struct
        self.states[state_id] = MDPState(id=state_id, continuous_center=center, labels=state_labels)
        
        # B. Store PRISM Groups (Inverted Index)
        for lbl in state_labels:
            self.labels[lbl].append(state_id)

    def to_prism(self, filename: str):
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        
        with open(filename, 'w') as f:
            f.write("// Auto-generated PRISM Model\n")
            f.write("mdp\n\n")
            
            # --- MODULE 1: PLANT ---
            f.write("module Plant\n")
            f.write(f"    s : [0..{self.num_states - 1}] init 0;\n\n")
            
            for (src_id, action), targets in self.transitions.items():
                if not targets: continue
                
                updates = []
                for t in targets:
                    p_min, p_max = min(t.p_min, t.p_max), max(t.p_min, t.p_max)
                    # PRISM Syntax: [p_min, p_max] : (s' = target)
                    updates.append(f"[{p_min:.6f}, {p_max:.6f}] : (s'={t.target_id})")
                
                f.write(f"    [{action}] s={src_id} -> " + " + ".join(updates) + ";\n")
                
            f.write("endmodule\n\n")
            
            # --- MODULE 2: CONTROLLER ---
            if self.controller_logic:
                f.write(self.controller_logic + "\n")

            # --- LABELS ---
            f.write("\n// --- LABELS ---\n")
            for label_name, states in self.labels.items():
                if not states: continue
                state_str = " | ".join([f"s={i}" for i in states])
                f.write(f'label "{label_name}" = ({state_str});\n')
                
        print(f"Successfully wrote {len(self.transitions)} transitions to {filename}.")

    def get_stats(self):
        return f"MDP(States={self.num_states}, Labels={list(self.labels.keys())})"