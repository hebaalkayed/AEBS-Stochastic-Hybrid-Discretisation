from abc import ABC, abstractmethod
from typing import List, Tuple

Scenario = Tuple[str, Tuple[float, float, float]]   # (name, (gap, v_ego, v_lead))


class LeadModel(ABC):
    name: str = "lead"
    prefix: str = "lead"

    @abstractmethod
    def vl_bounds(self) -> Tuple[float, float]: ...
    @abstractmethod
    def lead_noise_std(self) -> float: ...
    @abstractmethod
    def acceleration_choices(self) -> List[float]: ...
    @abstractmethod
    def next_v_lead(self, v_lead: float, a_lead: float, dt: float) -> float: ...
    @abstractmethod
    def scenarios(self) -> List[Scenario]: ...

    def is_nondeterministic(self) -> bool:
        return len(self.acceleration_choices()) > 1


class _HoldLead(LeadModel):
    """Shared base for constant-speed leads: hold v_lead (floored at 0), full
    (-1,31) grid, noise 2.0 — i.e. the committed a_lead=0 dynamics, unchanged."""
    def vl_bounds(self):            return (-1.0, 31.0)
    def lead_noise_std(self):       return 2.0
    def acceleration_choices(self): return [0.0]
    def next_v_lead(self, v_lead, a_lead, dt):
        return max(0.0, v_lead)


class StaticLead(_HoldLead):
    """Euro NCAP CCRs — stationary target. Identical dynamics to the committed
    'static' profile; scenarios start at v_lead=0."""
    name = "Static obstacle (Euro NCAP CCRs)"
    prefix = "static"
    def scenarios(self):
        return [
            ("Safe_Cruising",      (100.0, 15.0, 0.0)),
            ("Warning_Brake",      ( 30.0,  7.0, 0.0)),
            ("Emergency_Brake",    ( 10.0, 10.0, 0.0)),
            ("Imminent_Collision", (  5.0, 30.0, 0.0)),
            ("Post_Collision",     (  0.0,  0.0, 0.0)),
        ]


class SteadyLead(_HoldLead):
    """Euro NCAP CCRm — constant-speed lead. Same dynamics as static; scenarios
    start at v_lead = v0. Identical to the committed 'steady' profile (v0=20)."""
    name = "Steady lead at constant speed (Euro NCAP CCRm)"
    prefix = "steady"
    def scenarios(self):
        return [
            ("Following_Matched", (50.0, 20.0, 20.0)),
            ("Closing_Slow",      (40.0, 24.0, 20.0)),
            ("Closing_Warn",      (25.0, 27.0, 20.0)),
            ("Closing_Emergency", (10.0, 30.0, 20.0)),
        ]


class UnpredictableLead(LeadModel):
    """Adversarial / Euro NCAP CCRb lead. v_lead roams the full grid and the
    lead may apply any of several accelerations per step (nondeterministic).
    NOT generatable yet: needs a second action axis + nondeterministic Lead
    module in the PRISM composition. The wrapper rejects it until that lands."""
    name = "Unpredictable lead (nondeterministic)"
    prefix = "unpredictable"
    def __init__(self, v0=20.0, choices=(+2.0, 0.0, -4.0, -8.0), noise_std=2.0):
        self.v0 = float(v0); self._choices = list(choices); self._noise = float(noise_std)
    def vl_bounds(self):            return (-1.0, 31.0)
    def lead_noise_std(self):       return self._noise
    def acceleration_choices(self): return self._choices
    def next_v_lead(self, v_lead, a_lead, dt):
        return max(0.0, v_lead + a_lead * dt)
    def scenarios(self):
        v = self.v0
        return [("Closing_Warn", (25.0, v + 7, v)), ("Closing_Emergency", (10.0, v + 10, v))]


LEAD_MODELS = {
    "static": StaticLead(),
    "steady": SteadyLead(),
    # "unpredictable": UnpredictableLead(),   # enable when the nondet path is built
}


def get_lead_model(key_or_model):
    """Accept a LeadModel, or a string key into LEAD_MODELS."""
    if isinstance(key_or_model, LeadModel):
        return key_or_model
    if key_or_model not in LEAD_MODELS:
        raise KeyError(f"Unknown lead model '{key_or_model}'. Available: {sorted(LEAD_MODELS)}")
    return LEAD_MODELS[key_or_model]