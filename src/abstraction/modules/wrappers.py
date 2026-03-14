import numpy as np
from src.abstraction.types.interfaces import StochasticHybridSystem
from src.abstraction.algorithms.discretizer import compute_lq

class PlantWrapper(StochasticHybridSystem):
    """
    Wraps the physical plant for use by the discretization algorithm.

    State space: (gap_distance [m], v_ego [m/s], v_lead [m/s])
    Dimensions:  0 = gap (deterministic), 1 = v_ego (deterministic), 2 = v_lead (stochastic)

    Noise model:
        The lead vehicle acceleration is uncertain (Gaussian), which propagates as
        Gaussian noise on v_lead over one timestep:
            sigma_on_velocity = sigma_acceleration * dt

        Gap and v_ego are deterministic given the current state and action.
        Applying noise to these dimensions is physically wrong and causes
        probability mass to leak into incorrect cells.

    Noise level mapping (lead_noise_std in m/s²  →  sigma_on_velocity in m/s):
        lead_noise_std = 1.0  →  sigma_on_velocity ≈ 0.1 m/s   (low noise)
        lead_noise_std = 2.0  →  sigma_on_velocity ≈ 0.2 m/s   (baseline)
        lead_noise_std = 4.0  →  sigma_on_velocity ≈ 0.4 m/s   (high noise)

    NOTE: The previous low-noise experiment mistakenly used lead_noise_std=0.1,
    which gave sigma_on_velocity=0.01 — nearly deterministic, not gently reduced.
    """

    def __init__(self, plant, controller, lead_noise_std=2.0):
        """
        Args:
            plant:           The VehiclePlant instance.
            controller:      The AEBS controller instance.
            lead_noise_std:  Standard deviation of lead vehicle acceleration
                             uncertainty (m/s²).
        """
        self.plant = plant
        self.ctrl = controller

        # Combined acceleration sigma: sqrt(plant_noise² + lead_noise²)
        self.accel_sigma = np.sqrt(self.plant.noise_std**2 + lead_noise_std**2)

        # Velocity-domain sigma: sigma_accel * dt (one-step propagation)
        self.sigma_on_velocity = self.accel_sigma * self.plant.dt

        print(f"[PlantWrapper] lead_noise_std={lead_noise_std} m/s²  "
              f"→  sigma_on_velocity={self.sigma_on_velocity:.4f} m/s  "
              f"(kernel width on v_lead only, dims 0 and 1 are deterministic)")

    def get_action_space(self):
        return {
            0: self.ctrl.acc_coast,       # 0.0   m/s²
            1: self.ctrl.acc_brake,       # -4.0  m/s²
            2: self.ctrl.acc_emergency    # -9.8 or -8.0 m/s²
        }

    def get_next_state_distribution(self, s, a):
        """
        Returns (next_state_det, sigma_per_dim, L_t, L_q).

        sigma_per_dim = [0.0, 0.0, sigma_on_velocity]:
            - dim 0 (gap):   deterministic → sigma = 0, use indicator kernel
            - dim 1 (v_ego): deterministic → sigma = 0, use indicator kernel
            - dim 2 (v_lead): stochastic  → sigma > 0, use Gaussian CDF kernel

        L_q is computed from sigma_on_velocity (the noisy dimension only).
        Epsilon in the discretizer uses delta_vlead = resolution[2]/2, not
        a global mixed-unit cell_diameter, to keep epsilon ≈ 0.6 rather than 18.5.
        """
        next_state_det = self.plant.get_deterministic_next_state(s, a)

        # Per-dimension sigma: noise enters only through v_lead (index 2)
        sigma_per_dim = np.array([0.0, 0.0, self.sigma_on_velocity])

        L_t = self.plant.lipschitz_constant
        L_q = compute_lq(self.sigma_on_velocity)

        return next_state_det, sigma_per_dim, L_t, L_q


class ControllerWrapper(StochasticHybridSystem):
    # NOTE: No longer used by the pipeline (replaced by direct lookup table generation
    # to avoid the coordinate mapping bug). Kept to avoid breaking legacy imports.
    def __init__(self, controller):
        self.ctrl = controller
    def get_action_space(self): return {0: 0.0}
    def get_next_state_distribution(self, s, a):
        return np.array([0,0,0]), np.zeros(3), 0.0, 0.0