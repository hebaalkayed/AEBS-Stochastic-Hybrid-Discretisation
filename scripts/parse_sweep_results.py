"""Parse campaign Storm logs into one CSV and print a separation summary.
Usage: python3 scripts/parse_sweep_results.py logs/sweep_YYYYMMDD_HHMMSS
       python3 scripts/parse_sweep_results.py logs/horizon_YYYYMMDD_HHMMSS
Defaults to the most recent logs/sweep_* directory.
Rows pair the i-th property with the i-th Result line; if an invocation
aborted mid-batch, distrust the tail of that file.
"""
import csv
import glob
import os
import re
import sys


def main():
    if len(sys.argv) > 1:
        out_dir = sys.argv[1]
    else:
        candidates = glob.glob('logs/sweep_*')
        if not candidates:
            sys.exit("no logs/sweep_* directory found; pass one explicitly")
        out_dir = max(candidates, key=os.path.getmtime)

    rows = []
    for log in sorted(glob.glob(os.path.join(out_dir, '*imize_*.log'))):
        m = re.match(r'(maximize|minimize)_(.+)\.log', os.path.basename(log))
        if not m:
            continue
        direction, preset = m.group(1), m.group(2)
        props_name = ('pmax_' if direction == 'maximize' else 'pmin_') + preset + '.props'
        props = [p for p in open(os.path.join(out_dir, props_name)).read().splitlines()
                 if p.strip()]
        results = re.findall(r"^Result \(for '\"(.+?)\"' states\): (\S+)",
                             open(log).read(), re.M)
        if len(results) != len(props):
            print(f"WARNING {log}: {len(results)} results for {len(props)} properties")
        for prop, (flt, val) in zip(props, results):
            k = int(re.search(r'F<=(\d+)', prop).group(1))
            rows.append((preset, direction, k, flt, val))

    out_csv = os.path.join(out_dir, 'sweep_results.csv')
    with open(out_csv, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['preset', 'direction', 'horizon', 'filter', 'value'])
        w.writerows(rows)
    print(f"wrote {out_csv} ({len(rows)} rows)\n")

    table = {}
    for preset, direction, k, flt, val in rows:
        try:
            v = float(val)
        except ValueError:
            continue
        table.setdefault((preset, flt), {}).setdefault(k, {})[direction] = v

    print(f"{'preset':<14} {'filter':<24} {'largest k, Pmax<1':>20} {'largest k, informative':>24}")
    for (preset, flt), by_k in sorted(table.items()):
        k_pmax = max((k for k, d in by_k.items()
                      if d.get('maximize', 1.0) < 1.0), default=None)
        k_info = max((k for k, d in by_k.items()
                      if 0.0 < d.get('maximize', 1.0) < 1.0
                      or 0.0 < d.get('minimize', 0.0) < 1.0), default=None)
        print(f"{preset:<14} {flt:<24} {str(k_pmax):>20} {str(k_info):>24}")


if __name__ == '__main__':
    main()
