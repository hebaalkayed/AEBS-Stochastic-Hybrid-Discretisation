from abc import ABC, abstractmethod
from typing import List, Tuple


class LeadModel(ABC):
    """A lead-behaviour model owns the lead's DYNAMICS and their modelling
    envelope: the exogenous v_lead rule, the lead noise, the grid band, and
    the admissible acceleration choices. It owns nothing about verification
    scenarios; start states are ego-centric and live in
    src.abstraction.profiles.scenarios, resolved against the grid.
    """
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

    def is_nondeterministic(self) -> bool:
        return len(self.acceleration_choices()) > 1


class StaticLead(LeadModel):
    """Stationary/constant-speed lead under the committed a_lead = 0
    dynamics: hold v_lead, floored at zero. Full (-1, 31) grid band,
    lead-acceleration noise 2.0 m/s^2 (Assumption: lead-vehicle
    uncertainty)."""
    name = "Static obstacle (hold rule)"
    prefix = "static"

    def vl_bounds(self):            return (-1.0, 31.0)
    def lead_noise_std(self):       return 2.0
    def acceleration_choices(self): return [0.0]
    def next_v_lead(self, v_lead, a_lead, dt):
        return max(0.0, v_lead)


# A nondeterministic lead (several admissible accelerations per step,
# Pmax over worst-case lead behaviour) is the architecturally agreed future
# extension; it needs a second action axis in the discretiser and the
# composition, and the wrapper rejects any |choices| > 1 model until that
# path is built. No placeholder class is kept here: define it when the path
# exists.

LEAD_MODELS = {
    "static": StaticLead(),
}


def get_lead_model(key_or_model):
    """Accept a LeadModel, or a string key into LEAD_MODELS."""
    if isinstance(key_or_model, LeadModel):
        return key_or_model
    if key_or_model not in LEAD_MODELS:
        raise KeyError(f"Unknown lead model '{key_or_model}'. "
                       f"Available: {sorted(LEAD_MODELS)}")
    return LEAD_MODELS[key_or_model]