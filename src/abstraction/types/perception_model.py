class PerceptionModel:
    """
    Handles the interface between the Plant (s) and the Controller (y).
    Supports two modes:
    1. Perfect (Symbolic): Generates 1 line of code. Fast.
    2. Noisy (Explicit): Generates state-by-state map. Allows for noise.
    """
    def __init__(self, name="Perception", input_var="s", output_var="y", max_state=0):
        self.name = name
        self.input_var = input_var
        self.output_var = output_var
        self.max_state = max_state
        self.enable_noise = False
        self.labels = {} # Interface consistency

    def write_prism_body(self, f, sync_label="perceive"):
        f.write(f"    {self.output_var} : [0..{self.max_state}] init 0;\n")
        
        if not self.enable_noise:
            # --- MODE A: PERFECT PERCEPTION (Symbolic) ---
            f.write(f"    // Perfect Perception (Symbolic Optimization)\n")
            f.write(f"    [{sync_label}] true -> ({self.output_var}'={self.input_var});\n")
            
        else:
            # --- MODE B: NOISY PERCEPTION (Explicit) ---
            f.write(f"    // Noisy Perception (Explicit Enumeration)\n")
            for s_idx in range(self.max_state + 1):
                # Placeholder for Identity (Add Confusion Matrix math here later)
                f.write(f"    [{sync_label}] ({self.input_var}={s_idx}) -> ({self.output_var}'={s_idx});\n")