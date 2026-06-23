import numpy as np
from src.abstraction.types.interfaces import StochasticHybridSystem


class PlantWrapper(StochasticHybridSystem):
    """
    Wraps the physical plant for use by the discretisation algorithm.

    State space: (gap_distance [m], v_ego [m/s], v_lead [m/s])
    Dimensions:  0 = gap    (deterministic)
                 1 = v_ego  (deterministic)
                 2 = v_lead (stochastic — Gaussian noise from lead-vehicle uncertainty)

    The wrapper does two things: it forwards the deterministic next state from
    the concrete plant, and it attaches the per-dimension noise std (sigma) that
    the discretiser turns into the v_lead transition intervals.

    v_lead floor:
        The non-negativity floor on v_lead (a vehicle cannot travel backwards)
        is enforced inside the plant itself (vehicle_plant.get_deterministic_
        next_state returns max(0, v_lead)). The wrapper therefore does NOT clamp
        — there is nothing left to clamp. (An earlier version clamped here as a
        workaround for a plant-side coupling bug; that bug was fixed in the plant
        and the workaround removed.)

    Noise model:
        - Default (sigma_v_fn=None): constant sigma_on_velocity, derived from
          lead_noise_std and plant.noise_std combined via Pythagorean sum and
          scaled by plant.dt. Constant across the state space.
        - State-dependent (sigma_v_fn=callable): the caller supplies
          sigma_v_fn(v_lead) -> sigma_v [m/s], evaluated at the source state.
          NOTE: this makes sigma vary across a cell. The current discretiser
          kernel assumes a SINGLE sigma per cell and is only sound for constant
          noise; make_image_box() carries a guard that will raise if a
          state-dependent sigma is used without first extending box_prob_minmax
          to range over the sigma interval. The callable path is provided for
          the K-heterogeneity diagnostic (k_sampling.py), which does its own
          enumeration and does not go through the kernel.
    """

    def __init__(self, plant, controller, lead_noise_std=2.0, sigma_v_fn=None):
        self.plant = plant
        self.ctrl  = controller
        self.sigma_v_fn = sigma_v_fn  # callable v_lead -> sigma_v (m/s), or None for constant

        self.accel_sigma = np.sqrt(self.plant.noise_std**2 + lead_noise_std**2)
        self.sigma_on_velocity = self.accel_sigma * self.plant.dt

        if sigma_v_fn is None:
            print(f"[PlantWrapper] lead_noise_std={lead_noise_std} m/s²  "
                  f"->  sigma_on_velocity={self.sigma_on_velocity:.4f} m/s  "
                  f"(constant; kernel on v_lead only)")
        else:
            print(f"[PlantWrapper] state-dependent sigma_v provided  "
                  f"(sigma_v(0)={sigma_v_fn(0):.4f}, "
                  f"sigma_v(30)={sigma_v_fn(30):.4f} m/s)  "
                  f"(diagnostic use only — not sound through the current kernel)")

    def get_action_space(self):
        return {
            0: self.ctrl.acc_coast,       #  0.0  m/s²  coast
            1: self.ctrl.acc_brake,       # -4.0  m/s²  brake
            2: self.ctrl.acc_emergency    # -8.0 / -9.8 m/s²  emergency
        }

    def get_next_state_distribution(self, s, a):
        """
        Returns (next_state_det, sigma_per_dim).

        next_state_det : deterministic next (gap, v_ego, v_lead) from the plant.
                         v_lead is already floored at 0 inside the plant.
        sigma_per_dim  : [0.0, 0.0, sigma_v]; gap and v_ego are deterministic,
                         v_lead carries Gaussian noise. sigma_v is either the
                         constant self.sigma_on_velocity or self.sigma_v_fn(s[2]).
        """
        next_state_det = self.plant.get_deterministic_next_state(s, a)

        if self.sigma_v_fn is None:
            sigma_v = self.sigma_on_velocity
        else:
            sigma_v = self.sigma_v_fn(s[2])  # depends on source state's v_lead

        sigma_per_dim = np.array([0.0, 0.0, sigma_v])
        return next_state_det, sigma_per_dim


class ControllerWrapper(StochasticHybridSystem):
    # NOTE: No longer used by the pipeline. Kept to avoid breaking legacy imports.
    def __init__(self, controller):
        self.ctrl = controller

    def get_action_space(self):
        return {0: 0.0}

    def get_next_state_distribution(self, s, a):
        return np.array([0, 0, 0]), np.zeros(3)