import os

class DrnModelGenerator:
    """Direct DRN (Storm 'direct encoding') exporter of the CLOSED-LOOP
    composed IMDP: plant intervals with the quantised controller's action
    pinned at every state (perfect perception).

    WHY THIS EXISTS
        Storm's PRISM parser builds an in-memory AST of the whole file. On the
        29 GiB explicit artifact this inflated to ~500 GiB and was OOM-killed
        on the cluster node (memory.peak 537,882,292,224 bytes against
        527,799,936 kB MemTotal; all nine campaign runs died in the parse
        phase). Storm's DirectEncodingParser instead streams DRN line by line
        into the sparse matrix builder, so memory is bounded by the built
        model (~2M states, one choice each).

    WHAT IS EXPORTED (Form A: the composed product, phases collapsed)
        DRN has no module or synchronisation syntax; it is a flat listing, so
        the export target is the flattened synchronised product that Storm
        itself would build from the modular PRISM file. Under perfect
        perception that product collapses exactly: perceive is the identity
        (y' = s), control is deterministic (u' = ctrl(y)), so the action
        executed at plant state s is always ctrl(s), and the composed model
        is the plant IMDP with one action per state, the controller's. This
        file encodes precisely that. The modular PRISM artifact remains the
        source of truth for the architecture; this is its compilation for
        the checker, certified equivalent by check_drn_equivalence.py.

    HORIZON SEMANTICS (important)
        One DRN transition = one control period (dt). 'F<=100 "crash"' bounds
        100 physical steps = 10 s. Against the modular PRISM composition the
        same horizon needs 'F<=300', because each period costs three model
        transitions (perceive, control, time_step). Do not mix the two.

    TRACK 2 NOTE
        With noisy perception the collapse no longer applies; the export
        target then becomes the perception-aware composed IMDP that the
        pipeline's later construction step produces (the Track 3 box of the
        programme figure). This class stays a dumb translator of whatever
        composed flat IMDP it is handed.

    FORMAT PROVENANCE
        Pinned against storm 'stable' sources
        (src/storm-parsers/parser/DirectEncodingParser.cpp,
         src/storm-parsers/parser/ValueParser.cpp,
         src/storm/io/DirectEncodingExporter.cpp) and validated end to end on
        betel with a hand-computed 5-state toy IMDP (Storm 1.13.0 container;
        results 0.48 / 0.04 / 0.6; filters and multi-property batching
        confirmed). The binding constraints are:
          - header keywords: @type, @value_type, @parameters, @reward_models,
            @nr_states, @nr_choices, @model (note: @nr_*, not @nof_*)
          - state ids MUST be dense and consecutive from 0
          - transition line '<target> : <value>' with a space on each side of
            the colon
          - targets MUST be ascending within each action (sparse matrix
            builder ordering)
          - interval literal '[lo,hi]'; a bare number parses as a point
            interval

    VALUE FIDELITY
        Bound substrings are copied VERBATIM from the stored row strings of
        the in-memory IMDP (the same strings the PRISM exporter prints), so
        DRN and PRISM values are bitwise identical by construction, and the
        equivalence checker verifies it file against file. Outward rounding
        lives at the single formatting site in discretizer.py, upstream of
        BOTH exporters.

    SCENARIO ADDRESSING
        DRN has no 'const' mechanism, so '--constants start_s=ID' is replaced
        by per-scenario state labels ('scn_<name lowercased>') queried with
        property filters, e.g.
        filter(max, Pmax=? [ F<=100 "crash" ], "scn_emergency_brake").
        One state carries 'init' so the model has an initial state; the
        campaign itself addresses every scenario through filters and does not
        depend on which state is init.
    """

    def __init__(self, output_file):
        self.output_file = output_file

    def generate(self, imdp, controller_rules, scenario_ids=None,
                 init_state=None, validate_rows=True):
        """Stream the closed-loop composed IMDP to a DRN file.

        :param imdp: the plant IMDP (types.imdp.IMDP) with the sink finalised
        :param controller_rules: dict {state_id: action_id}, the quantised
               controller (ControllerModel.rules); must cover every state
               0..sink_id
        :param scenario_ids: dict {scenario_name: flat_state_id}; each becomes
               a per-state label 'scn_<name.lower()>'
        :param init_state: state id to carry the 'init' label; defaults to the
               first scenario id in the lead model's own catalogue, else 0
        :param validate_rows: if True, assert per row that targets are unique
               and that sum(lo) <= 1 <= sum(hi) to 1e-6 (well-formedness of
               the emitted file, not a soundness certificate)
        :returns: dict with n_states, n_choices, n_transitions
        """
        if imdp.sink_state_id is None:
            raise ValueError(
                "IMDP sink not finalised; call finalize_sink_state() before "
                "exporting, exactly as for the PRISM export.")

        n_states = imdp.sink_state_id + 1
        scenario_ids = scenario_ids or {}
        if init_state is None:
            init_state = next(iter(scenario_ids.values()), 0)

        print(f"[DRN] Translating closed-loop model to {self.output_file} "
              f"({n_states} states)...")
        outdir = os.path.dirname(self.output_file)
        if outdir:
            os.makedirs(outdir, exist_ok=True)

        # ---- pass 1: density and controller-coverage check --------------
        # The DRN parser requires ids 0..N-1 with no gaps and at least one
        # choice per state. Form A additionally requires the controller rule
        # to exist for every state and the plant row for that (state, action)
        # to exist. Assert rather than trust.
        for s in range(n_states):
            row = imdp.transitions.get(s)
            if not row:
                raise ValueError(
                    f"state {s} has no plant actions; empty rows deadlock "
                    f"the model in any format.")
            if s not in controller_rules:
                raise ValueError(
                    f"controller has no rule for state {s}; Form A export "
                    f"needs full coverage (ControllerModel.rules).")
            if controller_rules[s] not in row:
                raise ValueError(
                    f"controller action {controller_rules[s]} at state {s} "
                    f"has no plant row (available: {sorted(row)}).")
        n_choices = n_states  # one pinned action per state

        # ---- per-state labels -------------------------------------------
        # imdp.labels maps name -> set(state_id); invert lazily per state by
        # membership tests (the label count is small). Scenario labels and
        # 'init' are added on top.
        base_labels = dict(imdp.labels)
        for name, sid in scenario_ids.items():
            base_labels[f"scn_{name.lower()}"] = {sid}
        base_labels["init"] = {init_state}
        for name in base_labels:
            if any(c.isspace() for c in name):
                raise ValueError(f"label '{name}' contains whitespace; "
                                 f"quote-handling not implemented.")
        label_items = sorted(base_labels.items())

        n_transitions = 0
        with open(self.output_file, "w") as f:
            f.write("// Direct-encoding export of the CLOSED-LOOP composed "
                    "IMDP (plant + quantised controller, perfect "
                    "perception)\n")
            f.write("// one transition = one control period; F<=k bounds k "
                    "physical steps\n")
            f.write(f"// source: in-memory IMDP '{imdp.name}'; values "
                    f"verbatim from the row strings\n")
            f.write("@type: MDP\n")
            f.write("@value_type: double-interval\n")
            f.write("@parameters\n\n")
            f.write("@reward_models\n\n")
            f.write("@nr_states\n")
            f.write(f"{n_states}\n")
            f.write("@nr_choices\n")
            f.write(f"{n_choices}\n")
            f.write("@model\n")

            for s in range(n_states):
                labels = [name for name, states in label_items
                          if s in states]
                line = f"state {s}"
                if labels:
                    line += " " + " ".join(labels)
                out = [line]

                act = controller_rules[s]
                out.append(f"\taction {act}")
                parts = self._parse_row(imdp.transitions[s][act], s, act)
                parts.sort(key=lambda t: t[0])            # ascending targets
                if validate_rows:
                    self._validate_row(parts, s, act)
                for tgt, lo_str, hi_str in parts:
                    out.append(f"\t\t{tgt} : [{lo_str},{hi_str}]")
                n_transitions += len(parts)

                f.write("\n".join(out))
                f.write("\n")

                if s % 100000 == 0 and s > 0:
                    print(f"[DRN] ... {s}/{n_states} states", end="\r")

        print(f"\n[DRN] Export complete: {n_states} states, "
              f"{n_choices} choices (controller-pinned), "
              f"{n_transitions} transitions.")
        return {"n_states": n_states, "n_choices": n_choices,
                "n_transitions": n_transitions}

    @staticmethod
    def _parse_row(dist_str, s, act):
        """Split a stored row string into (target_id, lo_str, hi_str) tuples.

        Row strings are machine-generated with the fixed shape
            '[LO, HI] : (s'=T) + [LO, HI] : (s'=T) + ...'
        (discretizer aggregation and the sink self-loop both conform).
        Bound substrings are returned verbatim, not reparsed and reprinted,
        so the DRN carries bitwise the same decimal literals as the PRISM
        artifact.
        """
        parts = []
        for piece in dist_str.split(" + "):
            piece = piece.strip()
            try:
                close = piece.index("]")
                lo_str, hi_str = piece[1:close].split(",")
                tgt = int(piece[piece.rindex("'=") + 2: piece.rindex(")")])
            except (ValueError, IndexError) as e:
                raise ValueError(
                    f"unparseable update in row (s={s}, act={act}): "
                    f"'{piece}' ({e})")
            parts.append((tgt, lo_str.strip(), hi_str.strip()))
        return parts

    @staticmethod
    def _validate_row(parts, s, act, tol=1e-6):
        """Well-formedness of one emitted row: unique targets and a
        non-empty interval-distribution polytope, sum(lo) <= 1 <= sum(hi)."""
        tgts = [t for t, _, _ in parts]
        if len(tgts) != len(set(tgts)):
            raise ValueError(f"duplicate targets in row (s={s}, act={act})")
        lo_sum = sum(float(lo) for _, lo, _ in parts)
        hi_sum = sum(float(hi) for _, _, hi in parts)
        if lo_sum > 1.0 + tol or hi_sum < 1.0 - tol:
            raise ValueError(
                f"row (s={s}, act={act}) admits no distribution: "
                f"sum(lo)={lo_sum:.9f}, sum(hi)={hi_sum:.9f}")