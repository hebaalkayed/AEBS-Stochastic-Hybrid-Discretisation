from dataclasses import dataclass, field
from typing import Dict, List

@dataclass
class IntervalTransition:
    """Represents T(s, a, s') = [p_min, p_max]"""
    target_id: int
    p_min: float
    p_max: float

@dataclass
class IMDP:
    """
    The Mathematical Artifact produced by the Discretization Algorithm.
    Independent of PRISM syntax.
    """
    name: str
    num_states: int
    initial_state: int = 0
    # transitions[src][action] -> List[IntervalTransition]
    transitions: Dict[int, Dict[int, List[IntervalTransition]]] = field(default_factory=dict)
    
    def add_transition(self, src: int, action: int, target: int, p_min: float, p_max: float):
        if src not in self.transitions: self.transitions[src] = {}
        if action not in self.transitions[src]: self.transitions[src][action] = []
        self.transitions[src][action].append(IntervalTransition(target, p_min, p_max))

    def to_prism_module(self, module_name, sync_label, state_var="s", action_var="u") -> str:
        """Exports to PRISM module syntax."""
        lines = [f"\nmodule {module_name}"]
        lines.append(f"    {state_var} : [0..{self.num_states-1}] init {self.initial_state};")
        
        # Sort for stable output
        for src in sorted(self.transitions.keys()):
            for act, targets in sorted(self.transitions[src].items()):
                if not targets: continue
                
                updates = []
                for t in targets:
                    # Clamp probabilities to [0, 1]
                    p_min = max(0.0, min(1.0, t.p_min))
                    p_max = max(0.0, min(1.0, t.p_max))
                    updates.append(f"[{p_min:.5f}, {p_max:.5f}] : ({state_var}'={t.target_id})")
                
                # PRISM Guard: (s=src) & (u=act)
                guard = f"({state_var}={src})"
                if action_var: 
                    guard += f" & ({action_var}={act})"
                
                lines.append(f"    [{sync_label}] {guard} -> {' + '.join(updates)};")
        
        lines.append("endmodule")
        return "\n".join(lines)