import matplotlib.pyplot as plt

class SimulationPlotter:
    """
    Handles visualization of AEBS simulation results.
    Separates the 'Display' logic from the 'Physics' logic.
    """
    
    @staticmethod
    def plot_scenario(history, label, stop_idx=None, crash_idx=None):
        """
        Generates a standard 2-panel plot (Distance & Velocity).
        
        Args:
            history (dict): Must contain 'time', 'dist', 'ego_v', 'lead_v'
            label (str): Title for the plot
            stop_idx (int): Index where safe stop occurred (optional)
            crash_idx (int): Index where crash occurred (optional)
        """
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        
        # --- PLOT 1: DISTANCE ---
        ax1.plot(history['time'], history['dist'], color='blue', linewidth=2, label="Gap")
        ax1.axhline(0, color='red', linestyle='--', linewidth=1, label="Crash Line")
        
        # Annotation: STOPPED
        if stop_idx is not None:
            stop_t = history['time'][stop_idx]
            stop_d = history['dist'][stop_idx]
            ax1.plot(stop_t, stop_d, 'go') # Green Dot
            ax1.annotate(f'STOPPED\nGap: {stop_d:.2f}m', 
                         xy=(stop_t, stop_d), 
                         xytext=(stop_t+1, stop_d+10),
                         arrowprops=dict(facecolor='green', shrink=0.05))

        # Annotation: CRASHED
        if crash_idx is not None:
            crash_t = history['time'][crash_idx]
            ax1.plot(crash_t, 0, 'rx', markersize=12, markeredgewidth=3) # Red X
            ax1.text(crash_t + 0.5, 5, "IMPACT", color='red', fontweight='bold')

        ax1.set_ylabel('Distance (m)')
        ax1.set_title(f"Simulation Result: {label}")
        ax1.grid(True)
        ax1.legend()
        
        # --- PLOT 2: VELOCITY ---
        ax1_color = 'orange'
        ax2.plot(history['time'], history['ego_v'], color=ax1_color, linewidth=2, label="Ego Speed")
        
        if 'lead_v' in history and len(history['lead_v']) > 0:
            ax2.plot(history['time'], history['lead_v'], color='gray', linestyle='--', label="Lead Speed")
            
        ax2.axhline(0, color='black', linewidth=1)
        
        ax2.set_ylabel('Velocity (m/s)')
        ax2.set_xlabel('Time (s)')
        ax2.grid(True)
        ax2.legend()
        
        plt.tight_layout()
        plt.show()