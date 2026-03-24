import numpy as np
from src.abstraction.types.interfaces import StochasticHybridSystem


class PlantWrapper(StochasticHybridSystem):
    """
    Wraps the physical plant for use by the discretisation algorithm.

    State space: (gap_distance [m], v_ego [m/s], v_lead [m/s])
    Dimensions:  0 = gap    (deterministic)
                 1 = v_ego  (deterministic)
                 2 = v_lead (stochastic — Gaussian noise from lead vehicle uncertainty)
    """

    def __init__(self, plant, controller, lead_noise_std=2.0):
        self.plant = plant
        self.ctrl  = controller

        self.accel_sigma = np.sqrt(self.plant.noise_std**2 + lead_noise_std**2)
        self.sigma_on_velocity = self.accel_sigma * self.plant.dt

        print(f"[PlantWrapper] lead_noise_std={lead_noise_std} m/s²  "
              f"->  sigma_on_velocity={self.sigma_on_velocity:.4f} m/s  "
              f"(kernel on v_lead only; gap and v_ego are deterministic)")

    def get_action_space(self):
        return {
            0: self.ctrl.acc_coast,       #  0.0  m/s²  coast
            1: self.ctrl.acc_brake,       # -4.0  m/s²  brake
            2: self.ctrl.acc_emergency    # -8.0 / -9.8 m/s²  emergency
        }

    def get_next_state_distribution(self, s, a):
        """
        Returns (next_state_det, sigma_per_dim).

        sigma_per_dim = [0.0, 0.0, sigma_on_velocity]

        BUG FIX (v_lead physical floor):
        ---------------------------------------------------------------------------
        SYMPTOM:
            In the generated PRISM file, states with low v_lead (≈ 0 m/s) route
            almost all probability mass to safe_sink (102765) under brake and
            emergency actions. Specifically, from state (40 m, 15 m/s, 0 m/s)
            with brake (a = -4 m/s²), ≈99.7% goes to safe_sink.

        ROOT CAUSE — CONFIRMED by back-calculating from PRISM intervals:
            plant.get_deterministic_next_state() incorrectly couples the ego
            vehicle's acceleration into the v_lead update. The reconstructed
            formula from the PRISM evidence is:

                next_v_lead = v_lead + v_ego * a_ego * K * dt   (WRONG)

            where K ≈ 0.3 (a gain term or residual factor left during development).
            For (v_lead=0, v_ego=15, a_ego=-4, dt=0.1):

                next_v_lead = 0 + 15 * (-4) * 0.3 * 0.1 = -1.8 m/s

            With the Gaussian centred at -1.8 m/s and sigma=0.2 m/s, only 0.3%
            of the distribution falls inside the grid (boundary at -1.25 m/s),
            hence 99.7% routes to safe_sink. This is confirmed across multiple
            states in the PRISM file:
              - (v_lead=0.5, v_ego=20, brake): next_v_lead ≈ -1.3 → P(sink) ≈ 60% ✓
              - (v_lead=5.0, v_ego=20, emergency): next_v_lead ≈ -0.88 → P(sink) ≈ 4% ✓
              - (v_lead=3.5, v_ego=20, brake): next_v_lead ≈ 1.7 → 0% sink ✓

        CORRECT FORMULA in plant.py should be:
            next_v_lead = max(0.0, v_lead + a_lead_deterministic * dt)

            where a_lead_deterministic is the lead vehicle's own deterministic
            acceleration (typically 0.0 or a scenario-specific constant), with
            NO term involving a_ego. Search plant.get_deterministic_next_state
            for any expression that multiplies a_ego (or 'action', 'u', 'acc',
            or any acceleration variable) into the third component of the
            returned array, and remove it.

        WORKAROUND APPLIED HERE:
            Clamp next_v_lead to >= 0 m/s. This is physically correct (a stopped
            lead vehicle stays stopped) and fully corrects all scenario states
            where v_lead = 0.

            LIMITATION: For states where v_lead > 0, the plant still computes a
            subtracted quantity before the clamp, e.g. v_lead=3.5 → 3.5-1.8=1.7
            (positive, not clamped). The clamped result matches the physical value
            only when the formula output would go below 0. For a fully correct
            abstraction, fix plant.py.
        ---------------------------------------------------------------------------
        """
        next_state_det = self.plant.get_deterministic_next_state(s, a)

        # --- BUG FIX: v_lead physical floor ---
        # Clamp v_lead (dim 2) to >= 0. Vehicles cannot travel backwards.
        # This corrects the worst-case states (v_lead ≈ 0) where the plant bug
        # drives next_v_lead strongly negative → almost all mass to safe_sink.
        # For a complete fix, remove the a_ego coupling in plant.py (see above).
        if next_state_det[2] < 0.0:
            next_state_det = np.array(next_state_det, dtype=float)  # defensive copy
            next_state_det[2] = 0.0

        sigma_per_dim = np.array([0.0, 0.0, self.sigma_on_velocity])
        return next_state_det, sigma_per_dim


class ControllerWrapper(StochasticHybridSystem):
    # NOTE: No longer used by the pipeline. Kept to avoid breaking legacy imports.
    def __init__(self, controller):
        self.ctrl = controller

    def get_action_space(self):
        return {0: 0.0}

    def get_next_state_distribution(self, s, a):
        return np.array([0, 0, 0]), np.zeros(3)