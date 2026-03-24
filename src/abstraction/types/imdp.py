from collections import defaultdict

class IMDP:
    """
    A self-contained representation of an Interval Markov Decision Process.
    Holds logic, transitions, and semantics. 
    Updated to support 'Streaming' to prevent MemoryErrors.
    """
    def __init__(self, name, state_variable="s", action_variable="u"):
        self.name = name
        self.state_var = state_variable
        self.action_var = action_variable
        
        # Structure: {source_state: {action_id: "target_distribution_string"}}
        self.transitions = defaultdict(dict)
        
        # Semantics: {"label_name": {set_of_state_indices}}
        self.labels = defaultdict(set)
        
        self.initial_state = 0
        self.max_state_id = 0
        self.sink_state_id = None

    def add_transition(self, source, action, distribution_str):
        """Adds a transition logic to the model."""
        self.transitions[source][action] = distribution_str
        if source > self.max_state_id:
            self.max_state_id = source

    def add_label(self, label_name, state_id):
        self.labels[label_name].add(state_id)

    def add_bulk_label(self, label_name, state_ids):
        self.labels[label_name].update(state_ids)

    def finalize_sink_state(self):
        """Creates a self-looping sink state to catch out-of-bounds transitions."""
        self.sink_state_id = self.max_state_id + 1
        
        # Self-Loop with certainty
        sink_str = f"[1.0, 1.0] : ({self.state_var}'={self.sink_state_id})"
        
        # BUG FIX: Assign self-loop to ALL actions, not just action 0.
        # Collect every action id that appears anywhere in the model.
        all_actions = set()
        for src, actions in self.transitions.items():
            all_actions.update(actions.keys())
        # Fallback: if no transitions yet, at least cover action 0
        if not all_actions:
            all_actions = {0}
        
        for act_id in all_actions:
            self.transitions[self.sink_state_id][act_id] = sink_str
        
        # Label it structurally
        self.add_label("safe_sink", self.sink_state_id)
        
        print(f"[{self.name}] Finalized with Sink State at {self.state_var}={self.sink_state_id} "
              f"(actions: {sorted(all_actions)})")
        return self.sink_state_id

    def write_prism_body(self, f, sync_label="time_step"):
        """
        STREAM-WRITES the module body to the file handle 'f'.
        This is critical for avoiding MemoryErrors with large models.
        """
        # Define variable range including the sink state
        limit = self.sink_state_id if self.sink_state_id else self.max_state_id
        f.write(f"    {self.state_var} : [0..{limit}] init {self.initial_state};\n")
        
        # Write transitions line-by-line
        for src in sorted(self.transitions.keys()):
            actions = self.transitions[src]
            for act, dist in sorted(actions.items()):
                line = f"    [{sync_label}] ({self.state_var}={src}) & ({self.action_var}={act}) -> {dist};\n"
                f.write(line)