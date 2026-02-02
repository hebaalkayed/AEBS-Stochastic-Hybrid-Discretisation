import itertools

class ControllerModel:
    """
    Represents the deterministic logic of the AEBS Controller.
    Automatically compresses logic into ranges to minimize PRISM file size.
    """
    def __init__(self, name="Controller", input_var="y", output_var="u"):
        self.name = name
        self.input_var = input_var
        self.output_var = output_var
        self.rules = {} # {y_value: u_value}
        self.labels = {} # Interface consistency

    def add_rule(self, perceived_state, control_action):
        """Register a control decision: IF y = perceived_state THEN u' = control_action"""
        self.rules[perceived_state] = control_action

    def write_prism_body(self, f, sync_label="control"):
        """
        Stream-writes compressed PRISM logic to file handle 'f'.
        Uses Run-Length Encoding to group consecutive states.
        """
        f.write(f"    // Logic updates global {self.output_var} based on local {self.input_var}\n")
        f.write(f"    // Logic compressed from {len(self.rules)} explicit rules.\n")
        
        # 1. Sort by perceived state
        sorted_states = sorted(self.rules.keys())
        if not sorted_states:
            return
            
        # 2. Compress Logic
        ranges = [] # (start, end, action)
        
        current_start = sorted_states[0]
        current_end = sorted_states[0]
        current_action = self.rules[sorted_states[0]]

        for y in sorted_states[1:]:
            action = self.rules[y]
            
            # Check continuity (index + 1 AND same action)
            if (y == current_end + 1) and (action == current_action):
                current_end = y
            else:
                # Close range
                ranges.append((current_start, current_end, current_action))
                # Start new
                current_start = y
                current_end = y
                current_action = action
        
        # Final range
        ranges.append((current_start, current_end, current_action))

        # 3. Write Logic
        for (start, end, action) in ranges:
            if start == end:
                guard = f"({self.input_var}={start})"
            else:
                guard = f"({self.input_var}>={start}) & ({self.input_var}<={end})"
            
            f.write(f"    [{sync_label}] {guard} -> ({self.output_var}'={action});\n")