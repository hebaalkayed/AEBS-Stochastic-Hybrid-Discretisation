"""Composition certificate: closed-loop DRN artifact vs modular PRISM artifact.

PURPOSE
    The refactoring-soundness certificate for the DRN export path. The DRN
    encodes the flattened synchronised product of the modular PRISM model
    under perfect perception, with the three deterministic phases collapsed
    (one DRN transition = one control period). This script certifies that
    claim file against file, trusting neither exporter. Together with the
    collapse lemma (perceive is the identity and control is deterministic,
    so the action executed at plant state s is ctrl(s)), a pass transfers
    the modular model's semantics, and hence the containment certificate of
    the underlying IMDP values, to the DRN artifact.

WHAT IS CHECKED
    1. Perception module is the symbolic identity ([perceive] true ->
       (y'=s);). If the explicit noisy enumeration is present instead, the
       collapse is invalid and the check FAILS by design.
    2. Controller map: the run-length-encoded [control] rules of the PRISM
       file are parsed into a full state -> action map.
    3. Transitions, in lockstep and constant memory: the DRN must contain
       exactly one action per state; that action must equal the controller
       map at the state; and its transition list must equal, target for
       target and parsed-float bound for bound, the plant module's
       [time_step] row for (state, action) in the PRISM file.
    4. Controller coverage: every plant state seen must have a controller
       rule, and the DRN must have exactly one row per state 0..N-1.
    5. Labels: the state set of every label present in both files must be
       identical. Labels present only in the DRN are accepted when expected
       ('init' and 'scn_*', which replace the PRISM route's --constants
       scenario selection).
    6. A SHA-256 over the canonical composed-row stream of each side,
       recorded so a passing run can be quoted in artifact metadata.

USAGE (on betel, inside tmux; streams both files once, one to two hours at
       production scale, constant memory apart from label sets)
    python3 drn_equivalence_test.py \
        --prism artifacts/modular_system.prism \
        --drn   artifacts/modular_system.drn
"""

import argparse
import hashlib
import re
import sys

PRISM_ROW = re.compile(
    r"^\s*\[time_step\]\s*\(s=(\d+)\)\s*&\s*\(u=(\d+)\)\s*->\s*(.*);\s*$")
PRISM_LABEL = re.compile(r'^label\s+"([^"]+)"\s*=\s*(.*);\s*$')
CTRL_SINGLE = re.compile(
    r"^\s*\[control\]\s*\(y=(\d+)\)\s*->\s*\(u'=(\d+)\);\s*$")
CTRL_RANGE = re.compile(
    r"^\s*\[control\]\s*\(y>=(\d+)\)\s*&\s*\(y<=(\d+)\)\s*->\s*\(u'=(\d+)\);\s*$")
PERCEIVE_IDENTITY = re.compile(
    r"^\s*\[perceive\]\s*true\s*->\s*\(y'=s\);\s*$")
PERCEIVE_EXPLICIT = re.compile(r"^\s*\[perceive\]\s*\(s=\d+\)")


# ---------------------------------------------------------------------
#  PRISM side
# ---------------------------------------------------------------------
def prism_controller_and_perception(path):
    """One pass collecting the controller ranges and the perception form.

    Returns (ranges, identity_ok) where ranges is a sorted list of
    (start, end, action) covering the controller's run-length encoding.
    Kept as ranges, not expanded, so lookup is O(log R) and memory is tiny.
    """
    ranges = []
    identity = False
    explicit = False
    with open(path, "r", buffering=1 << 22) as f:
        for line in f:
            m = CTRL_SINGLE.match(line)
            if m:
                y, u = int(m.group(1)), int(m.group(2))
                ranges.append((y, y, u))
                continue
            m = CTRL_RANGE.match(line)
            if m:
                ranges.append((int(m.group(1)), int(m.group(2)),
                               int(m.group(3))))
                continue
            if PERCEIVE_IDENTITY.match(line):
                identity = True
            elif PERCEIVE_EXPLICIT.match(line):
                explicit = True
    ranges.sort()
    return ranges, identity, explicit


def ctrl_lookup(ranges, s):
    import bisect
    i = bisect.bisect_right(ranges, (s, float("inf"), float("inf"))) - 1
    if i >= 0:
        start, end, u = ranges[i]
        if start <= s <= end:
            return u
    return None


def prism_rows(path):
    """Yield (src, act, [(tgt, lo, hi)]) from the plant module, in file
    order (ascending src, then act, as written by IMDP.write_prism_body)."""
    with open(path, "r", buffering=1 << 22) as f:
        for line in f:
            m = PRISM_ROW.match(line)
            if not m:
                continue
            src, act, rhs = int(m.group(1)), int(m.group(2)), m.group(3)
            parts = []
            for piece in rhs.split(" + "):
                piece = piece.strip()
                close = piece.index("]")
                lo, hi = piece[1:close].split(",")
                tgt = int(piece[piece.rindex("'=") + 2: piece.rindex(")")])
                parts.append((tgt, float(lo), float(hi)))
            parts.sort(key=lambda t: t[0])
            yield src, act, parts


def prism_labels(path):
    """Parse the label lines into {name: set(states)} from their
    range-encoded guards."""
    labels = {}
    single = re.compile(r"\(s=(\d+)\)")
    ranged = re.compile(r"\(s>=(\d+)\)&\(s<=(\d+)\)")
    with open(path, "r", buffering=1 << 22) as f:
        for line in f:
            m = PRISM_LABEL.match(line)
            if not m:
                continue
            name, logic = m.group(1), m.group(2)
            states = set()
            for part in logic.split(" | "):
                part = part.strip()
                ms = single.fullmatch(part)
                mr = ranged.fullmatch(part)
                if ms:
                    states.add(int(ms.group(1)))
                elif mr:
                    states.update(range(int(mr.group(1)),
                                        int(mr.group(2)) + 1))
                else:
                    raise ValueError(
                        f"unparseable label clause for '{name}': '{part}'")
            labels[name] = states
    return labels


# ---------------------------------------------------------------------
#  DRN side
# ---------------------------------------------------------------------
def drn_rows_and_labels(path):
    """Single streaming pass over the DRN. Returns a generator of
    (state, [(action, parts)]) and a dict filled with label sets as a side
    effect; the generator must be exhausted before the labels are complete,
    which the lockstep comparison guarantees."""
    label_sets = {}

    def rows():
        src = None
        acts = []          # [(action_id, [(tgt, lo, hi)])] for current state
        in_model = False
        with open(path, "r", buffering=1 << 22) as f:
            for line in f:
                if not in_model:
                    if line.startswith("@model"):
                        in_model = True
                    continue
                if line.startswith("state "):
                    if src is not None:
                        yield src, acts
                    tokens = line.split()
                    src = int(tokens[1])
                    for lab in tokens[2:]:
                        label_sets.setdefault(lab, set()).add(src)
                    acts = []
                elif line.startswith("\taction"):
                    acts.append((int(line.split()[1]), []))
                elif line.startswith("\t\t"):
                    body = line.strip()
                    tgt_str, val = body.split(" : ")
                    val = val.strip()
                    if val.startswith("["):
                        lo, hi = val[1:-1].split(",")
                    else:
                        lo = hi = val
                    acts[-1][1].append((int(tgt_str), float(lo), float(hi)))
            if src is not None:
                yield src, acts

    return rows(), label_sets


# ---------------------------------------------------------------------
#  Comparison
# ---------------------------------------------------------------------
def canonical_hash_update(h, src, act, parts):
    h.update(f"{src}|{act}|".encode())
    for tgt, lo, hi in parts:
        h.update(f"{tgt}:{lo!r}:{hi!r};".encode())


def report_row_diff(pp, dp):
    pset = dict((t, (l, h)) for t, l, h in pp)
    dset = dict((t, (l, h)) for t, l, h in dp)
    only_p = sorted(set(pset) - set(dset))
    only_d = sorted(set(dset) - set(pset))
    if only_p:
        print(f"  targets only in PRISM: {only_p[:10]}"
              f"{' ...' if len(only_p) > 10 else ''}")
    if only_d:
        print(f"  targets only in DRN:   {only_d[:10]}"
              f"{' ...' if len(only_d) > 10 else ''}")
    for t in sorted(set(pset) & set(dset)):
        if pset[t] != dset[t]:
            print(f"  target {t}: PRISM {pset[t]} vs DRN {dset[t]}")
            break


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--prism", required=True)
    ap.add_argument("--drn", required=True)
    ap.add_argument("--skip-labels", action="store_true",
                    help="compare transitions only")
    args = ap.parse_args()

    # ---- pass 0: controller map and perception form from the PRISM file
    ranges, identity, explicit = prism_controller_and_perception(args.prism)
    if explicit or not identity:
        print("FAIL: perception module is not the symbolic identity "
              "([perceive] true -> (y'=s);). The Form A collapse is only "
              "valid under perfect perception; export the perception-aware "
              "composed IMDP instead.")
        sys.exit(1)
    print(f"[check] perception: symbolic identity confirmed")
    print(f"[check] controller: {len(ranges)} run-length ranges parsed")

    # ---- lockstep: DRN states vs controller-selected PRISM plant rows
    drn_gen, drn_label_sets = drn_rows_and_labels(args.drn)
    p_iter = prism_rows(args.prism)
    p_buffer = {}          # (src, act) -> parts, for the current source only
    p_current_src = None
    p_pushback = None
    p_hash, d_hash = hashlib.sha256(), hashlib.sha256()
    n_states = 0
    n_trans = 0
    expected_next = 0

    def prism_rows_for(src):
        """Collect all plant rows of the given source from the ordered
        stream; constant memory (rows of one source at a time)."""
        nonlocal p_pushback
        rows = {}
        while True:
            if p_pushback is not None:
                item = p_pushback
                p_pushback = None
            else:
                item = next(p_iter, None)
            if item is None:
                return rows
            s, a, parts = item
            if s < src:
                # plant rows for a state the DRN skipped: only legal if the
                # DRN is missing states, caught by the density check below
                continue
            if s > src:
                p_pushback = item
                return rows
            rows[a] = parts

    for src, acts in drn_gen:
        if src != expected_next:
            print(f"FAIL: DRN states not dense: expected {expected_next}, "
                  f"got {src}")
            sys.exit(1)
        expected_next += 1

        if len(acts) != 1:
            print(f"FAIL: DRN state {src} has {len(acts)} actions; the "
                  f"closed-loop artifact must have exactly one "
                  f"(controller-pinned).")
            sys.exit(1)
        d_act, d_parts = acts[0]
        d_parts = sorted(d_parts)

        c_act = ctrl_lookup(ranges, src)
        if c_act is None:
            print(f"FAIL: controller map has no rule for state {src}")
            sys.exit(1)
        if d_act != c_act:
            print(f"FAIL: state {src}: DRN pins action {d_act}, controller "
                  f"module says {c_act}")
            sys.exit(1)

        prows = prism_rows_for(src)
        if c_act not in prows:
            print(f"FAIL: PRISM plant has no row (s={src}, u={c_act}); "
                  f"available actions: {sorted(prows)}")
            sys.exit(1)
        p_parts = prows[c_act]
        if p_parts != d_parts:
            print(f"FAIL: composed row s={src} (action {c_act}) differs.")
            report_row_diff(p_parts, d_parts)
            sys.exit(1)

        canonical_hash_update(p_hash, src, c_act, p_parts)
        canonical_hash_update(d_hash, src, d_act, d_parts)
        n_states += 1
        n_trans += len(d_parts)
        if n_states % 200000 == 0:
            print(f"[check] ... {n_states} states, {n_trans} transitions",
                  end="\r", flush=True)

    # any plant rows beyond the last DRN state means the DRN is truncated
    leftover = next(p_iter, None) if p_pushback is None else p_pushback
    if leftover is not None and leftover[0] >= n_states:
        print(f"FAIL: PRISM has plant rows for state {leftover[0]} beyond "
              f"the DRN's last state {n_states - 1} (truncated DRN?)")
        sys.exit(1)

    print(f"\n[check] composition PASS: {n_states} states, {n_trans} "
          f"transitions; every DRN row equals the controller-selected "
          f"PRISM plant row")
    print(f"[check] canonical sha256 (PRISM, composed): {p_hash.hexdigest()}")
    print(f"[check] canonical sha256 (DRN):             {d_hash.hexdigest()}")
    assert p_hash.hexdigest() == d_hash.hexdigest()

    if not args.skip_labels:
        expected_drn_only = lambda n: n == "init" or n.startswith("scn_")
        plabels = prism_labels(args.prism)
        ok = True
        for name in sorted(set(plabels) | set(drn_label_sets)):
            ps = plabels.get(name)
            ds = drn_label_sets.get(name)
            if ps is None:
                if expected_drn_only(name):
                    print(f"[check] label '{name}': DRN-only as expected "
                          f"({len(ds)} state)")
                else:
                    print(f"FAIL: label '{name}' only in DRN "
                          f"({len(ds)} states)")
                    ok = False
            elif ds is None:
                print(f"FAIL: label '{name}' only in PRISM "
                      f"({len(ps)} states)")
                ok = False
            elif ps != ds:
                print(f"FAIL: label '{name}' differs: |PRISM|={len(ps)}, "
                      f"|DRN|={len(ds)}, symmetric difference {len(ps ^ ds)}")
                ok = False
            else:
                print(f"[check] label '{name}' PASS ({len(ps)} states)")
        if not ok:
            sys.exit(1)

    print("[check] COMPOSITION CERTIFICATE PASS: the DRN encodes the "
          "closed-loop product of the modular PRISM model under perfect "
          "perception (one transition per control period).")


if __name__ == "__main__":
    main()