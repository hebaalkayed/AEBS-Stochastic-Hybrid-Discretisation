import os

class PrismModelGenerator:
    """
    A 'Dumb' Translator.
    It does NOT create logic. It only prints what is inside the Module objects.
    """
    def __init__(self, output_file):
        self.output_file = output_file

    # UPDATED SIGNATURE: Added 'constants'
    def generate(self, modules, globals_dict=None, constants=None):
        """
        :param modules: List of objects (IMDP, ControllerModel, etc.)
        :param globals_dict: Dictionary of global variables {name: range_str}
        :param constants: Dictionary of constants {name: type} e.g. {'start_s': 'int'}
        """
        print(f"[Generator] Translating model to {self.output_file}...")
        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
        
        with open(self.output_file, 'w') as f:
            # Pass constants to the header writer
            self._write_header(f, globals_dict, constants)
            self._write_turn_module(f)
            
            all_labels = {}
            
            for module in modules:
                self._write_module(f, module)
                
                if hasattr(module, 'labels'):
                    for name, states in module.labels.items():
                        if name not in all_labels:
                            all_labels[name] = set()
                        all_labels[name].update(states)

            self._write_labels(f, all_labels)
            
        print(f"[Generator] Export Complete.")

    # UPDATED HEADER WRITER
    def _write_header(self, f, globals_dict, constants):
        f.write("// --- MODULAR AEBS MODEL (Robust IMDP) ---\n")
        f.write("mdp\n\n")
        
        # Write Constants (New Feature)
        if constants:
            for name, dtype in constants.items():
                f.write(f"const {dtype} {name};\n")
        
        # Write Globals
        if globals_dict:
            for name, defn in globals_dict.items():
                f.write(f"global {name} : {defn};\n")
        f.write("\n")

    def _write_turn_module(self, f):
        # FIX: Reordered so controller acts BEFORE the first physics step.
        # Old order: time_step → perceive → control (cold start: u=0 applied before controller sees state)
        # New order: perceive → control → time_step (controller reads state and sets u first)
        f.write("module Turn\n")
        f.write("    t : [1..3] init 1;\n") 
        f.write("    [perceive]  (t=1) -> (t'=2);\n")
        f.write("    [control]   (t=2) -> (t'=3);\n")
        f.write("    [time_step] (t=3) -> (t'=1);\n")
        f.write("endmodule\n\n")

    def _write_module(self, f, module):
        f.write(f"module {module.name}\n")
        
        sync_label = "time_step" 
        if module.name == "Controller": sync_label = "control"
        if module.name == "Perception": sync_label = "perceive"
        
        if hasattr(module, 'write_prism_body'):
            module.write_prism_body(f, sync_label)
        else:
            f.write(module.get_prism_body(sync_label))
            
        f.write("\nendmodule\n\n")

    def _write_labels(self, f, all_labels):
        f.write("// --- SEMANTIC LABELS ---\n")
        state_var = "s" 
        
        for name in sorted(all_labels):
            states = all_labels[name]
            if not states:
                continue

            sorted_states = sorted(list(states))
            ranges = []
            
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
            
            logic_parts = []
            for (start, end) in ranges:
                if start == end:
                    logic_parts.append(f"({state_var}={start})")
                else:
                    logic_parts.append(f"({state_var}>={start})&({state_var}<={end})")
            
            full_logic = " | ".join(logic_parts)
            f.write(f'label "{name}" = {full_logic};\n')