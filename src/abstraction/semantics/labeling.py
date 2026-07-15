class Labels:
    CRASH = "crash"
    SAFE = "safe"
    BRAKING = "braking"
    EMERGENCY = "emergency"
    NOMINAL = "nominal"


class LabelingGrammar:
    """Region labels for the abstraction, bound to the controller the
    abstraction is built against.

    The labels are DUAL-TRIGGER RISK REGIONS (distance OR TTC), not
    controller-action regions: the controller's decision logic branches on
    v_rel, so e.g. a state can carry 'emergency' while the controller
    commands only warn-level braking there. Results prose must therefore
    say 'the warning region' / 'the emergency region', never 'states where
    the controller (emergency-)brakes'.

    Threshold binding is dynamic and FAIL-LOUD: the numbers come from the
    controller instance, so the labels always match the controller in the
    pipeline, and a controller without thresholds raises instead of
    silently borrowing another mode's numbers.

    Shipped modes of AEBSController:
      mode='industry' (benchmark-backed):
          dist_warn=10.0, dist_emergency=2.0, ttc_warn=2.6, ttc_emergency=1.0
      mode='safe' (early-braking variant):
          dist_warn=15.0, dist_emergency=5.0, ttc_warn=6.0, ttc_emergency=4.0
    """

    def __init__(self, controller):
        self.dist_warn = controller.dist_warn
        self.dist_crit = controller.dist_emergency
        self.ttc_warn = controller.ttc_warn
        self.ttc_crit = controller.ttc_emergency

    def get_labels(self, state):
        """Labels satisfied by state (gap, v_ego, v_lead).

        nominal = safe and outside the warning region ('no need to brake');
        the emergency region is a subset of the warning region (its
        thresholds are strictly tighter), so nominal excludes both.
        """
        gap, v_ego, v_lead = state[0], state[1], state[2]
        v_rel = v_ego - v_lead

        props = set()

        if gap <= 0:
            return [Labels.CRASH]
        props.add(Labels.SAFE)

        # TTC from closing speed, same guard as the controller
        ttc = gap / v_rel if v_rel > 0.1 else 999.0

        if (gap < self.dist_warn) or (ttc < self.ttc_warn):
            props.add(Labels.BRAKING)

        if (gap < self.dist_crit) or (ttc < self.ttc_crit):
            props.add(Labels.EMERGENCY)

        if Labels.BRAKING not in props:
            props.add(Labels.NOMINAL)

        return list(props)