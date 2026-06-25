import numpy as np
from src.abstraction.types.interfaces import StochasticHybridSystem


class PlantWrapper(StochasticHybridSystem):
    """
    Wraps the plant for the discretiser. Forwards the deterministic next state
    and attaches the per-dimension noise std (sigma) on v_lead.

    State space: (gap, v_ego, v_lead);  dims 0,1 deterministic, dim 2 stochastic.

    LEAD INJECTION:
        - lead_model=<LeadModel> (the path the pipeline now uses): the wrapper
          asks the LeadModel for v_lead_next (the exogenous lead rule), passes it
          to the plant, and takes the noise from the model's lead_noise_std.
        - lead_model=None: kept only as a low-level fallback that runs the plant's
          own default lead rule (max(0, v_lead)) with the lead_noise_std argument.
          No production caller reaches this since the pipeline is injected-only;
          it remains for unit-level use of the wrapper in isolation.

    A nondeterministic lead (|choices|>1) is rejected: it needs a second action
    axis and a nondeterministic Lead module in the composition (not built yet).
    """

    def __init__(self, plant, controller, lead_noise_std=2.0, sigma_v_fn=None,
                 lead_model=None):
        self.plant = plant
        self.ctrl  = controller
        self.sigma_v_fn = sigma_v_fn
        self.lead_model = lead_model

        if lead_model is not None:
            if lead_model.is_nondeterministic():
                raise NotImplementedError(
                    f"Lead model '{lead_model.name}' is nondeterministic "
                    f"(choices={lead_model.acceleration_choices()}). The discretiser "
                    f"and PRISM composition do not yet support a second (lead) "
                    f"action axis. Build the nondeterministic-lead path first.")
            self._a_lead = lead_model.acceleration_choices()[0]
            lead_noise_std = lead_model.lead_noise_std()   # model owns the noise

        self.accel_sigma = np.sqrt(self.plant.noise_std**2 + lead_noise_std**2)
        self.sigma_on_velocity = self.accel_sigma * self.plant.dt

        if sigma_v_fn is None:
            tag = f"lead_model='{lead_model.name}'  " if lead_model is not None else ""
            print(f"[PlantWrapper] {tag}lead_noise_std={lead_noise_std} m/s²  "
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
        Returns (next_state_det, sigma_per_dim). With an injected lead_model the
        lead rule comes from the model; with lead_model=None it falls back to the
        plant's default lead rule.
        """
        if self.lead_model is None:
            next_state_det = self.plant.get_deterministic_next_state(s, a)
        else:
            v_lead_next = self.lead_model.next_v_lead(s[2], self._a_lead, self.plant.dt)
            next_state_det = self.plant.get_deterministic_next_state(s, a, v_lead_next=v_lead_next)

        if self.sigma_v_fn is None:
            sigma_v = self.sigma_on_velocity
        else:
            sigma_v = self.sigma_v_fn(s[2])

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