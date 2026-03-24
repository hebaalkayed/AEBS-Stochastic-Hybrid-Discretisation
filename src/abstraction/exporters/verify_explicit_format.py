"""
verify_explicit_format.py
=========================
Writes a tiny hand-crafted explicit IMDP and checks that PRISM 4.8 can:
  1. Parse it without OOM
  2. Return a sensible Pmax result

Run this BEFORE convert_to_explicit.py to confirm the interval syntax
(lo:hi vs lo hi) that your PRISM version accepts.

Usage:
    python verify_explicit_format.py [path_to_prism_binary]

Default prism binary: ~/prism-4.8.1-linux64-x86/bin/prism
"""

import os
import sys
import subprocess
import tempfile

PRISM_BIN = (sys.argv[1] if len(sys.argv) > 1
             else os.path.expanduser("~/prism-4.8.1-linux64-x86/bin/prism"))

# ── Tiny IMDP: 4 states ───────────────────────────────────────────────────────
# State 0: crash (absorbing, label "crash")
# State 1: normal, 2 actions
#   u=0 (coast): -> 0 with [0.05:0.15], -> 1 with [0.85:0.95]
#   u=1 (brake): -> 0 with [0.01:0.05], -> 1 with [0.95:0.99]
# State 2: safe sink (absorbing)
# State 3: init state, always -> state 1

TRA_CONTENT_COLON = """\
4 6 9
0 0 0 1.0:1.0
1 0 0 0.05000000:0.15000000
1 0 1 0.85000000:0.95000000
1 1 0 0.01000000:0.05000000
1 1 1 0.95000000:0.99000000
2 0 2 1.0:1.0
3 0 1 1.0:1.0
"""

TRA_CONTENT_SPACE = """\
4 6 9
0 0 0 1.0 1.0
1 0 0 0.05000000 0.15000000
1 0 1 0.85000000 0.95000000
1 1 0 0.01000000 0.05000000
1 1 1 0.95000000 0.99000000
2 0 2 1.0 1.0
3 0 1 1.0 1.0
"""

LAB_CONTENT = """\
#DECLARATION
crash safe
#END
0: 0
1: 1
2: 1
3: 1
"""

PCTL_QUERY = 'Pmax=? [ F "crash" ]'


def run_test(label, tra_content, stem):
    tra_path = stem + ".tra"
    lab_path = stem + ".lab"

    with open(tra_path, "w") as f:
        f.write(tra_content)
    with open(lab_path, "w") as f:
        f.write(LAB_CONTENT)

    cmd = [
        PRISM_BIN,
        "-importmodel", stem + ".all",
        "-imdp",
        "-pctl", PCTL_QUERY,
        "-const", "start_s=3",  # unused here but harmless
        "-javamaxmem", "4g",
    ]

    print(f"\n[Test: {label}]")
    print(f"  Command: {' '.join(cmd)}")

    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=120
        )
        stdout = result.stdout + result.stderr

        if "Result:" in stdout:
            for line in stdout.splitlines():
                if "Result:" in line:
                    print(f"  ✓ PASS — {line.strip()}")
                    return True
        elif "Exception" in stdout or "Error" in stdout:
            for line in stdout.splitlines():
                if "Exception" in line or "Error" in line or "error" in line:
                    print(f"  ✗ FAIL — {line.strip()}")
            return False
        else:
            print(f"  ? UNKNOWN — last lines of output:")
            for line in stdout.splitlines()[-5:]:
                print(f"    {line}")
            return False

    except subprocess.TimeoutExpired:
        print("  ✗ TIMEOUT")
        return False
    except FileNotFoundError:
        print(f"  ✗ PRISM not found at: {PRISM_BIN}")
        print("    Pass the path as argument: python verify_explicit_format.py /path/to/prism")
        return False


def main():
    print("="*60)
    print("  PRISM Explicit IMDP Format Verification")
    print(f"  PRISM binary: {PRISM_BIN}")
    print("="*60)

    with tempfile.TemporaryDirectory() as tmpdir:
        stem_colon = os.path.join(tmpdir, "test_colon")
        stem_space = os.path.join(tmpdir, "test_space")

        ok_colon = run_test("lo:hi syntax", TRA_CONTENT_COLON, stem_colon)
        ok_space = run_test("lo hi syntax", TRA_CONTENT_SPACE, stem_space)

    print("\n" + "="*60)
    if ok_colon:
        print("  RESULT: Use lo:hi syntax in convert_to_explicit.py")
        print("          (no changes needed — this is the default)")
    elif ok_space:
        print("  RESULT: Use 'lo hi' (space-separated) syntax")
        print("          Edit convert_to_explicit.py line:")
        print("            tmp_f.write(f\"{src} {action} {tgt} {lo:.8f}:{hi:.8f}\\n\")")
        print("          Change to:")
        print("            tmp_f.write(f\"{src} {action} {tgt} {lo:.8f} {hi:.8f}\\n\")")
    else:
        print("  RESULT: Neither format worked.")
        print("  Options:")
        print("  1. Check PRISM binary path")
        print("  2. Check PRISM version supports -imdp flag (needs 4.7+)")
        print("  3. Try Storm instead (see README below)")
        print()
        print("  Storm alternative:")
        print("    storm --explicit model.tra model.lab --imdp \\")
        print("          --prop 'Pmax=? [ F \"crash\" ]'")
    print("="*60 + "\n")


if __name__ == "__main__":
    main()