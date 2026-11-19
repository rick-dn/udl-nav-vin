#!/usr/bin/env python3
"""
regression.py - Replays the regression cases and checks nothing got worse

For each case in regression.yaml: replay the bag (with its fault scenario, if
any) through replay_run.sh, score it with score_run.py, and compare the
metrics with the case's limits. Prints a table and exits 1 if any case fails
or doesn't run.

Runs go to runs/eval/reg_<stamp>_<case>/, and a summary (CSV + JSON) to
runs/regression/<stamp>/. Needs the sim and any other OpenVINS stopped.
Takes about 15 minutes for all cases.

Usage: python3 regression.py [--only easy medium] [--config regression.yaml]
"""

import argparse
import csv
import json
import subprocess
import sys
import time
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
VIN_DIR = HERE.parents[2]
EVAL_DIR = VIN_DIR / 'runs' / 'eval'
OUT_DIR = VIN_DIR / 'runs' / 'regression'
BAG_DIR = VIN_DIR / 'runs' / 'bags'
METRICS = ['ate_rmse_m', 'ate_max_m', 'scale_error_pct']


def run_case(case, stamp):
    run = f'reg_{stamp}_{case["name"]}'
    cmd = ['bash', str(HERE / 'replay_run.sh'), case['bag'], run]
    if case.get('faults'):
        cmd += ['--faults', case['faults']]
    log = OUT_DIR / stamp / f'{case["name"]}.log'
    with open(log, 'w') as f:
        rc = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT).returncode
        if rc == 0:
            rc = subprocess.run(['python3', str(HERE / 'score_run.py'), run],
                                stdout=f, stderr=subprocess.STDOUT).returncode
    metrics_file = EVAL_DIR / run / 'metrics.json'
    if rc != 0 or not metrics_file.exists():
        return run, None, f'did not run (see {log})'
    return run, json.loads(metrics_file.read_text()), None


def check(metrics, limits):
    """List of 'metric value > limit' strings for every limit exceeded."""
    fails = []
    for k, lim in limits.items():
        v = metrics.get(k)
        if v is None:
            fails.append(f'{k} missing')
        elif abs(v) > lim:
            fails.append(f'{k} {v:.3f} > {lim}')
    return fails


def main():
    parser = argparse.ArgumentParser(description='Run the regression cases')
    parser.add_argument('--config', default=str(HERE / 'regression.yaml'))
    parser.add_argument('--only', nargs='+', help='run only these case names')
    args = parser.parse_args()

    cases = yaml.safe_load(open(args.config))['cases']
    if args.only:
        unknown = set(args.only) - {c['name'] for c in cases}
        if unknown:
            sys.exit(f'unknown case(s): {", ".join(sorted(unknown))}')
        cases = [c for c in cases if c['name'] in args.only]
    missing = [c['bag'] for c in cases if not (BAG_DIR / c['bag'] / 'metadata.yaml').exists()]
    if missing:
        sys.exit(f'missing bag(s) in {BAG_DIR}: {", ".join(sorted(set(missing)))}')

    stamp = time.strftime('%Y%m%d_%H%M%S')
    (OUT_DIR / stamp).mkdir(parents=True)
    rows = []
    start = time.monotonic()
    for i, case in enumerate(cases, 1):
        print(f'[{i}/{len(cases)}] {case["name"]}: {case["bag"]}'
              + (f' + {case["faults"]}' if case.get('faults') else ''), flush=True)
        run, m, err = run_case(case, stamp)
        fails = [err] if err else check(m, case.get('limits', {}))
        rows.append({'case': case['name'], 'run': run, 'pass': not fails,
                     **{k: (m or {}).get(k) for k in METRICS},
                     'baseline_ate_rmse_m': case.get('baseline', {}).get('ate_rmse_m'),
                     'limit_ate_rmse_m': case.get('limits', {}).get('ate_rmse_m'),
                     'why': '; '.join(fails)})
        print(f'      {"PASS" if not fails else "FAIL"}'
              + (f'  ATE {m["ate_rmse_m"]:.3f} m' if m else '')
              + (f'  ({rows[-1]["why"]})' if fails else ''), flush=True)

    # summary
    with open(OUT_DIR / stamp / 'summary.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    (OUT_DIR / stamp / 'summary.json').write_text(json.dumps(rows, indent=2))

    def fmt(v, p=3):
        return '-' if v is None else f'{v:.{p}f}'

    print(f'\n{"case":18s} {"result":6s} {"ATE":>8s} {"baseline":>9s} {"limit":>7s} {"max":>8s} {"scale%":>7s}')
    for r in rows:
        print(f'{r["case"]:18s} {"pass" if r["pass"] else "FAIL":6s} {fmt(r["ate_rmse_m"]):>8s} '
              f'{fmt(r["baseline_ate_rmse_m"]):>9s} {fmt(r["limit_ate_rmse_m"]):>7s} '
              f'{fmt(r["ate_max_m"]):>8s} {fmt(r["scale_error_pct"], 1):>7s}')
    n_fail = sum(not r['pass'] for r in rows)
    mins = (time.monotonic() - start) / 60
    print(f'\n{len(rows) - n_fail}/{len(rows)} passed in {mins:.0f} min. Summary: {OUT_DIR / stamp}')
    sys.exit(1 if n_fail else 0)


if __name__ == '__main__':
    main()
