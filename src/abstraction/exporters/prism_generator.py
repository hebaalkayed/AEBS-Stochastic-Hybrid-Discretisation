import os

class PrismModelGenerator:
    """
    A 'Dumb' Translator.
    It does NOT create logic. It only prints what is inside the Module objects.
    """
    def __init__(self, output_file):
        self.output_file = output_file

    def generate(self, modules, globals_dict=None):
        """
        :param modules: List of objects (IMDP, ControllerModel, etc.)
        :param globals_dict: Dictionary of global variables {name: range_str}
        """
        print(f"[Generator] Translating model to {self.output_file}...")
        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
        
        with open(self.output_file, 'w') as f:
            self._write_header(f, globals_dict)
            self._write_turn_module(f)
            
            # The Generator simply iterates over the provided full logic blocks
            all_labels = {}
            
            for module in modules:
                self._write_module(f, module)
                
                # Collect labels to write at the end of the file
                if hasattr(module, 'labels'):
                    for name, states in module.labels.items():
                        if name not in all_labels:
                            all_labels[name] = set()
                        all_labels[name].update(states)

            self._write_labels(f, all_labels)
            
        print(f"[Generator] Export Complete.")

    def _write_header(self, f, globals_dict):
        f.write("// --- MODULAR AEBS MODEL (Robust IMDP) ---\n")
        f.write("mdp\n\n")
        if globals_dict:
            for name, defn in globals_dict.items():
                f.write(f"global {name} : {defn};\n")
        f.write("\n")

    def _write_turn_module(self, f):
        f.write("module Turn\n")
        f.write("    [time_step] (t=1) -> (t'=2);\n")
        f.write("    [perceive]  (t=2) -> (t'=3);\n")
        f.write("    [control]   (t=3) -> (t'=1);\n")
        f.write("endmodule\n\n")

    def _write_module(self, f, module):
        """Delegates body generation to the module object."""
        f.write(f"module {module.name}\n")
        
        # Determine sync label based on module name convention
        sync_label = "time_step" 
        if module.name == "Controller": sync_label = "control"
        if module.name == "Perception": sync_label = "perceive"
        
        # --- FIX 1: USE STREAMING WRITE INSTEAD OF GET ---
        if hasattr(module, 'write_prism_body'):
            module.write_prism_body(f, sync_label)
        else:
            # Fallback for older legacy objects if any
            f.write(module.get_prism_body(sync_label))
            
        f.write("\nendmodule\n\n")

    def _write_labels(self, f, all_labels):
        f.write("// --- SEMANTIC LABELS ---\n")
        # Assume labels map to Plant State 's'
        state_var = "s" 
        
        for name, states in all_labels.items():
            if not states:
                continue

            # --- FIX 2: INTERVAL COMPRESSION FOR LABELS ---
            sorted_states = sorted(list(states))
            ranges = []
            
            # Logic to find consecutive runs (1, 2, 3 -> 1..3)
            if len(sorted_states) > 0:
                start = sorted_states[0]
                prev = sorted_states[0]
                
                for s in sorted_states[1:]:
                    if s == prev + 1:
                        prev = s
                    else:
                        ranges.append((start, prev))
                        start = s
                        prev = s
                ranges.append((start, prev))
            
            # Format logic string
            logic_parts = []
            for (start, end) in ranges:
                if start == end:
                    logic_parts.append(f"({state_var}={start})")
                else:
                    logic_parts.append(f"({state_var}>={start})&({state_var}<={end})")
            
            full_logic = " | ".join(logic_parts)
            f.write(f'label "{name}" = {full_logic};\n')