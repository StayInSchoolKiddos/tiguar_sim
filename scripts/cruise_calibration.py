#!/usr/bin/env python3
"""Fit, stage, validate and report a fleet-average cruise propulsion calibration."""
import hashlib
import html
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from datetime import datetime, timezone

os.environ.setdefault('MPLCONFIGDIR', '/tmp/tiguar-mpl')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.optimize import brentq

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / '.deps'))
import jsbsim
from calibrate import prop_tables, add_physics, D, MASS, G, K

COLUMNS = ['green', 'red', 'yellow_new_uprop', 'yellow_old_uprop']
REPORT = ROOT / 'reports/cruise_calibration'
BASELINE = ROOT / 'config/log_calibration_baseline.json'
CANDIDATE = ROOT / 'config/cruise_calibration.json'
MODEL = 'tig_cruise_candidate'
RHO_CONVERSION = 515.378818


def read_targets(path):
    frame = pd.read_csv(path)
    if list(frame.columns) != ['thr_bin'] + COLUMNS:
        raise ValueError('Expected thr_bin and the four named aircraft columns')
    bounds = frame.thr_bin.str.extract(r'^(\d+(?:\.\d+)?)-(\d+(?:\.\d+)?)$').astype(float)
    if bounds.isna().any().any() or frame.empty:
        raise ValueError('Invalid or empty throttle bins')
    low, high = bounds[0].to_numpy(), bounds[1].to_numpy()
    if not (np.all(low >= 0) and np.all(high <= 100) and np.all(high > low)
            and np.all(low[1:] >= high[:-1])):
        raise ValueError('Throttle bins must be ordered, disjoint and within 0–100')
    rpm = frame[COLUMNS].apply(pd.to_numeric, errors='raise').to_numpy()
    if not np.isfinite(rpm).all() or (rpm <= 0).any():
        raise ValueError('RPM values must be finite and positive for all four aircraft')
    frame['throttle'] = (low + high) / 200
    frame['rpm_mean'] = rpm.mean(axis=1)
    frame['rpm_min'] = rpm.min(axis=1)
    frame['rpm_max'] = rpm.max(axis=1)
    return frame


def fit_cruise(source, output):
    baseline_path = BASELINE if BASELINE.exists() else ROOT / 'config/calibration.json'
    baseline = json.loads(baseline_path.read_text())
    if baseline.get('cruise'):
        raise ValueError('Original log calibration is required as baseline')
    targets = read_targets(source)
    rho = float(pd.read_csv(ROOT / 'analysis/2026-05-01/segments.csv').rho.median())
    cp = prop_tables()[1]
    targets['advance_ratio'] = 23 / (targets.rpm_mean / 60 * D)
    targets['supported'] = targets.advance_ratio.between(cp[0, 0], cp[-1, 0])
    targets['reason'] = np.where(targets.supported, 'supported', 'outside propeller coefficient domain')
    supported = targets[targets.supported]
    if len(supported) < 2:
        raise ValueError('At least two supported cruise points are required')
    t = supported.throttle.to_numpy()
    n = supported.rpm_mean.to_numpy() / 60
    power = rho * n**3 * D**5 * np.interp(23 / (n * D), cp[:, 0], cp[:, 1])
    # Constrained monotone least-squares fit; current input is already monotone.
    from scipy.optimize import isotonic_regression
    power = isotonic_regression(power).x
    if not np.all(power > 0):
        raise ValueError('Nonpositive cruise shaft power')
    old_t = np.array(baseline['throttle_grid'])
    old_p = np.array(baseline['shaft_power_w'])
    lower = old_t[(old_t > 0) & (old_t < t[0])]
    # Preserve the shape of the existing low-power response, scaled to the join.
    join_power = np.interp(t[0], old_t, old_p)
    lower_power = np.interp(lower, old_t, old_p) * power[0] / join_power
    grid = np.r_[0, lower, t, 1]
    watts = np.r_[0, lower_power, power, power[-1]]
    config = dict(baseline)
    config['baseline_log_scores'] = config.pop('scores')
    config.pop('power_coefficients_w', None)
    config.update(throttle_grid=grid.tolist(), shaft_power_w=watts.tolist())
    config['cruise'] = {
        'source': str(source.resolve().relative_to(ROOT)),
        'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
        'columns': COLUMNS, 'weighting': 'equal columns, equal bins',
        'reference_tas_mps': 23, 'reference_rho_kg_m3': rho,
        'throttle_interpretation': 'actual output fraction; bin midpoint',
        'targets': targets.to_dict('records'),
        'acceptance': {'rms_relative_error_pct': 3, 'max_relative_error_pct': 5},
        'status': 'unvalidated',
    }
    config['assumptions'] = [a for a in config['assumptions'] if 'Power outside' not in a]
    config['assumptions'] += [
        'Fleet-average cruise fit at provisional TAS 23 m/s; engine and propeller RPM assumed equal.',
        'Four columns weighted equally, including both yellow propeller configurations; bin sample counts/statistic unknown.',
        'Reference density is median May 1 segment density; spreadsheet density is unknown.',
        'Low-power baseline shape scaled to first supported cruise point; zero command gives zero power.',
        'Power held constant from highest bin midpoint to full throttle; this endpoint is unmeasured.',
        'Low-RPM windmilling outside the supplied propeller coefficient domain is not calibrated.',
    ]
    output.write_text(json.dumps(config, indent=2) + '\n')
    return config, targets


def bench(model, speed, altitude):
    f = jsbsim.FGFDMExec(str(ROOT))
    f.set_debug_level(0)
    output_dir = ROOT/'runtime/cruise_bench'
    output_dir.mkdir(parents=True, exist_ok=True)
    f.set_output_path(str(output_dir))
    f.set_aircraft_path('aircraft')
    f.set_engine_path(f'aircraft/{model}/Engine')
    if not f.load_model(model):
        raise RuntimeError(f'Cannot load {model}')
    f.disable_output()
    f['ic/h-sl-ft'] = altitude
    f.set_dt(.001)
    f.run_ic()
    # Stationary airframe in a headwind: constant propeller inflow, live rotor dynamics.
    f['forces/hold-down'] = 1
    f['atmosphere/wind-north-fps'] = -speed / .3048
    return f


def settle(f, throttle):
    f['fcs/throttle-cmd-norm'] = float(throttle)
    previous = None
    stable = 0
    for second in range(30):
        for _ in range(1000):
            if not f.run():
                raise RuntimeError('JSBSim stopped during propulsion sweep')
        rpm = f['propulsion/engine/propeller-rpm']
        if not np.isfinite(rpm):
            return rpm, False, second + 1
        stable = stable + 1 if previous is not None and abs(rpm - previous) < .1 else 0
        if stable >= 2:
            return rpm, True, second + 1
        previous = rpm
    return rpm, False, 30


def sweep(targets, rho):
    def density(altitude):
        f = bench('tig_cruise_baseline', 23, altitude)
        return f['atmosphere/rho-slugs_ft3'] * RHO_CONVERSION - rho
    altitude = brentq(density, -1000, 20000, xtol=.001)
    rows = []
    for model in ['tig_calibrated', MODEL]:
        for speed in [21, 23, 25]:
            f = bench('tig_cruise_baseline' if model == 'tig_calibrated' else model, speed, altitude)
            for row in targets.itertuples():
                rpm, converged, seconds = settle(f, row.throttle)
                actual_speed = f['velocities/vtrue-fps'] * .3048
                actual_rho = f['atmosphere/rho-slugs_ft3'] * RHO_CONVERSION
                if abs(actual_speed - speed) > .01 or abs(actual_rho - rho) > 1e-4:
                    raise RuntimeError('Propulsion bench did not maintain reference inflow/density')
                rows.append(dict(model=model, tas=speed, throttle=row.throttle,
                                 thr_bin=row.thr_bin, target_rpm=row.rpm_mean,
                                 rpm=rpm, converged=converged, settle_seconds=seconds,
                                 reference_supported=row.supported,
                                 advance_ratio=speed / (rpm / 60 * D),
                                 error_pct=100 * (rpm / row.rpm_mean - 1)))
    return pd.DataFrame(rows), altitude


def compare_logs(config, baseline):
    rows = []
    cp = prop_tables()[1]
    ct = prop_tables()[0]
    for date in ['2026-05-01', '2026-04-30']:
        windows = add_physics(pd.read_csv(ROOT / f'analysis/{date}/segments.csv'))
        windows = windows[windows.prop_in_range].copy()
        for label, c in [('baseline', baseline), ('cruise', config)]:
            predictions = []
            for row in windows.itertuples():
                power = np.interp(row.throttle_output_norm, c['throttle_grid'], c['shaft_power_w'])
                def residual(n):
                    return row.rho * n**3 * D**5 * np.interp(row.tas / (n * D), cp[:, 0], cp[:, 1]) - power
                minimum = row.tas / (cp[-1, 0] * D)
                if residual(minimum) > 0 or residual(200) < 0:
                    predictions.append((float('nan'), float('nan')))
                    continue
                n = brentq(residual, minimum, 200)
                thrust = c['thrust_scale'] * row.rho * n*n * D**4 * np.interp(row.tas/(n*D), ct[:, 0], ct[:, 1])
                drag = row.qS * (c['cd0'] + K * row.CL**2)
                climb = (thrust-drag-MASS*row.accel)*row.tas/(MASS*G)
                predictions.append((n*60, climb))
            windows[f'{label}_rpm'] = [p[0] for p in predictions]
            windows[f'{label}_climb'] = [p[1] for p in predictions]
        windows['date'] = date
        rows.append(windows)
    return pd.concat(rows, ignore_index=True)


def command(args, log):
    result = subprocess.run([sys.executable] + args, cwd=ROOT, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    log.write_text(result.stdout)
    return result.returncode == 0


def report(config, baseline, targets, swept, logs, validation, run_dir):
    REPORT.mkdir(parents=True, exist_ok=True)
    targets.to_csv(REPORT / 'fleet_targets.csv', index=False)
    swept.to_csv(REPORT / 'propulsion_sweep.csv', index=False)
    logs.to_csv(REPORT / 'flight_log_comparison.csv', index=False)
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    x = targets.throttle * 100
    ax = axes[0, 0]
    ax.fill_between(x, targets.rpm_min, targets.rpm_max, color='gray', alpha=.2, label='Four-column range')
    ax.plot(x, targets.rpm_mean, 'ko-', label='Fleet average')
    excluded = ~targets.supported
    ax.scatter(x[excluded], targets.rpm_mean[excluded], marker='x', s=100, color='red', label='Excluded targets')
    for model, label in [('tig_calibrated', 'Before'), (MODEL, 'Candidate')]:
        s = swept[(swept.model == model) & (swept.tas == 23)]
        ax.plot(s.throttle*100, s.rpm, '--', label=label)
    ax.set(title='23 m/s: measured targets and simulated RPM', ylabel='RPM')
    ax = axes[0, 1]
    for c, label in [(baseline, 'Before'), (config, 'Candidate')]:
        ax.plot(np.array(c['throttle_grid'])*100, c['shaft_power_w'], 'o-', label=label)
    ax.set(title='Throttle-to-shaft-power mapping', ylabel='Shaft power (W)')
    ax = axes[1, 0]
    for speed in [21, 23, 25]:
        s = swept[(swept.model == MODEL) & (swept.tas == speed)]
        ax.plot(s.throttle*100, s.rpm, 'o-', label=f'{speed} m/s TAS')
        outside = s[s.advance_ratio > prop_tables()[1][-1, 0]]
        ax.scatter(outside.throttle*100, outside.rpm, marker='x', color='red', zorder=4)
    ax.plot(x, targets.rpm_mean, 'k:', label='Fleet targets at nominal 23 m/s')
    ax.set(title='Speed sensitivity (red ×: outside propeller table)', ylabel='RPM')
    ax = axes[1, 1]
    for model, label in [('tig_calibrated', 'Before'), (MODEL, 'Candidate')]:
        s = swept[(swept.model == model) & (swept.tas == 23) & swept.reference_supported]
        ax.plot(s.throttle*100, s.error_pct, 'o-', label=label)
    ax.axhspan(-5, 5, color='green', alpha=.1, label='±5% per-bin criterion')
    ax.set(title='Supported cruise-bin error', ylabel='RPM error (%)')
    for ax in axes.flat:
        ax.set_xlabel('Throttle output (%)'); ax.grid(alpha=.25); ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(REPORT/'cruise_fit.png', dpi=160); plt.close(fig)
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    metrics = []
    for date, group in logs.groupby('date'):
        # Common supported windows avoid comparing differently selected populations.
        group = group.dropna(subset=['baseline_rpm', 'cruise_rpm'])
        for label in ['baseline', 'cruise']:
            axes[0].scatter(group.rpm, group[f'{label}_rpm'], label=f'{date} {label}', s=18)
            axes[1].scatter(group.climb, group[f'{label}_climb'], label=f'{date} {label}', s=18)
            metrics.append(dict(date=date, model=label, common_windows=len(group),
                                rpm_rmse=float(np.sqrt(np.mean((group[f'{label}_rpm']-group.rpm)**2))),
                                climb_rmse_mps=float(np.sqrt(np.mean((group[f'{label}_climb']-group.climb)**2)))))
    axes[0].plot([3000, 7500], [3000, 7500], 'k:'); axes[0].set(xlabel='Logged RPM', ylabel='Predicted RPM')
    axes[1].plot([-2, 3], [-2, 3], 'k:'); axes[1].set(xlabel='Logged climb (m/s)', ylabel='Predicted climb (m/s)')
    for ax in axes: ax.grid(alpha=.25); ax.legend(fontsize=7)
    fig.tight_layout(); fig.savefig(REPORT/'log_comparison.png', dpi=160); plt.close(fig)
    telemetry = run_dir/'telemetry.csv'
    sitl_image = ''
    phase_table = ''
    if telemetry.exists():
        flight = pd.read_csv(telemetry)
        phases = []
        for seq, altitude, speed in [(3,100,23),(5,100,25),(7,130,23),(8,100,23),(9,100,23)]:
            window = flight[flight.mission_seq == seq]
            window = window[window.t > window.t.max()-20]
            if not window.empty:
                phases.append(dict(mission_seq=seq, altitude_target_m=altitude,
                                   altitude_mean_m=float(window.alt.mean()),
                                   airspeed_target_mps=speed, airspeed_mean_mps=float(window.eas.mean()),
                                   throttle_mean_pct=float(window.throttle_pct.mean())))
        validation['settled_phases'] = phases
        phase_frame = pd.DataFrame(phases)
        phase_frame.to_csv(REPORT/'sitl_phases.csv', index=False)
        phase_table = phase_frame.to_html(index=False, float_format=lambda v: f'{v:.2f}')
        fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        for ax, column, label in zip(axes, ['alt', 'eas', 'throttle_pct'], ['Altitude (m)', 'Airspeed (m/s)', 'Throttle (%)']):
            if column in flight: ax.plot(flight.t, flight[column])
            ax.set_ylabel(label); ax.grid(alpha=.25)
        axes[-1].set_xlabel('Simulation time (s)')
        fig.tight_layout(); fig.savefig(REPORT/'sitl_profile.png', dpi=160); plt.close(fig)
        sitl_image = ('<h2>Fresh coupled flight</h2><p>Last 20 seconds of each commanded hold. '
                      'Airspeed here is simulator telemetry EAS; the propulsion bench uses TAS.</p>'
                      + phase_table + '<img src="sitl_profile.png">'
                      '<p>The trace retains startup telemetry, including the brief pre-takeoff airspeed spike. '
                      'Throttle and airspeed oscillations follow takeoff, speed changes, and the turn reversal; '
                      'they settle before the acceptance windows. Passing these steady-hold checks does not '
                      'validate transient engine response. Controller gains were not retuned.</p>')
    table = pd.DataFrame(metrics)
    table.to_csv(REPORT/'log_metrics.csv', index=False)
    validation['log_metrics'] = metrics
    (REPORT/'validation.json').write_text(json.dumps(validation, indent=2)+'\n')
    nominal = swept[(swept.model == MODEL) & (swept.tas == 23)]
    comparison = targets[['thr_bin','rpm_mean','rpm_min','rpm_max','supported']].copy()
    comparison['candidate_rpm'] = nominal.rpm.to_numpy()
    comparison['error_pct'] = nominal.error_pct.to_numpy()
    comparison.to_csv(REPORT/'bin_comparison.csv', index=False)
    excluded_bins = ', '.join(targets.loc[~targets.supported, 'thr_bin'])
    first = targets[targets.supported].iloc[0]
    log_findings = []
    for date in ['2026-05-01', '2026-04-30']:
        before = table[(table.date == date) & (table.model == 'baseline')].iloc[0]
        after = table[(table.date == date) & (table.model == 'cruise')].iloc[0]
        log_findings.append(f'<li>{date}: RPM RMSE changes from {before.rpm_rmse:.0f} to {after.rpm_rmse:.0f} RPM; climb RMSE changes from {before.climb_rmse_mps:.2f} to {after.climb_rmse_mps:.2f} m/s. These comparisons do not meet a claim of improved agreement with the original flights.</li>')
    report_html = f'''<!doctype html><html lang="en"><meta charset="utf-8">
<title>Tiguar fleet cruise calibration</title><style>body{{font:16px system-ui;max-width:1100px;margin:40px auto;padding:0 20px;line-height:1.5}}img{{width:100%}}table{{border-collapse:collapse;font-size:14px}}td,th{{padding:6px;border:1px solid #ccc}}pre{{white-space:pre-wrap}}</style>
<h1>Tiguar fleet-average cruise calibration</h1>
<p><strong>Result: {html.escape(validation['decision'])}</strong></p>
<p>Four columns have equal weight. Targets use throttle-bin midpoints; the unknown per-bin sample counts and statistic prevent statistical confidence claims. Both yellow configurations count as separate columns.</p>
<p>Reference: 23 m/s provisional TAS, density {config['cruise']['reference_rho_kg_m3']:.4f} kg/m³ from May 1 windows. Engine RPM is assumed equal to propeller RPM. Supported nominal-bin RMS relative error: {validation['rms_error_pct']:.3f}%; maximum absolute error: {validation['max_error_pct']:.3f}%. These are fit-point consistency checks, not independent accuracy estimates: the power table was constructed from these same targets.</p>
<img src="cruise_fit.png"><h2>What the results show</h2>
<ul><li>The first supported fleet target is {first.thr_bin}% throttle ({first.rpm_mean:.0f} RPM). Excluded bins: {excluded_bins}%. Their advance ratios exceed the supplied propeller table. Simulated values there are diagnostics, not validated windmilling behavior.</li>
<li>Maximum shaft power changes from {baseline['shaft_power_w'][-1]:.0f} W to {config['shaft_power_w'][-1]:.0f} W. The highest measurement represents the 80–100% bin midpoint (90%), so 90–100% power is held constant provisionally.</li>
<li>The gray band is aircraft/configuration spread, not measurement uncertainty. Matching the average does not reproduce every aircraft.</li>
<li>The 21–25 m/s curves show aerodynamic load sensitivity at fixed throttle and density. They are predictions; the spreadsheet has no speed-resolved observations to validate them.</li>
<li>Low-throttle baseline shape is rescaled to meet the first supported target. Idle, windmilling, fuel use and throttle transients remain uncalibrated.</li>
<li>Thrust coefficients and airframe drag remain fixed. Better spreadsheet RPM agreement alone does not establish better thrust or climb accuracy.</li></ul>
<h2>Comparison with existing flight logs</h2>
<p>Both models are scored on the same supported windows. These logs are not independent fleet validation: aircraft identity and overlap with spreadsheet data are unknown. April 30 was held out from the original log fit. Review any increase in error as a tradeoff of replacing that aircraft/day fit with a fleet mean.</p>
<ul>{''.join(log_findings)}</ul>
{table.to_html(index=False, float_format=lambda v: f'{v:.2f}')}<img src="log_comparison.png">
{sitl_image}<h2>Per-bin results at 23 m/s</h2>{comparison.to_html(index=False, float_format=lambda v: f'{v:.2f}')}
<h2>Validation and reproduction</h2><pre>{html.escape(json.dumps(validation, indent=2))}</pre>
<p>Reproduce with <code>MPLCONFIGDIR=/tmp/tiguar-mpl python3 scripts/cruise_calibration.py</code>. Validation uses the actual rotor inertia and 1 ms time steps in a stationary-airframe headwind bench. Each throttle point must settle to less than 0.1 RPM change for two consecutive one-second intervals, within 30 seconds.</p>
<p>Data: <a href="fleet_targets.csv">fleet targets</a>, <a href="propulsion_sweep.csv">propulsion sweep</a>, <a href="flight_log_comparison.csv">log predictions</a>, <a href="validation.json">validation</a>, <a href="pipeline_checks.txt">pipeline checks</a>.</p></html>'''
    (REPORT/'index.html').write_text(report_html)


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, help='Reuse a completed SITL run only if its model and parameters match this candidate')
    args = parser.parse_args()
    REPORT.mkdir(parents=True, exist_ok=True)
    if not BASELINE.exists():
        shutil.copy2(ROOT/'config/calibration.json', BASELINE)
    baseline = json.loads(BASELINE.read_text())
    config, targets = fit_cruise(ROOT/'rpm_cruise_by_aircraft.csv', CANDIDATE)
    # The before curve must always be the archived log fit, including on reruns.
    baseline_model = ROOT/'aircraft/tig_calibrated'
    backup = ROOT/'runtime/cruise_default_backup'
    if backup.exists():
        shutil.rmtree(backup)
    shutil.copytree(baseline_model, backup)
    # Build the baseline in a separate named model without touching the default.
    baseline_name = 'tig_cruise_baseline'
    if not command(['scripts/build_candidate.py','--config',str(BASELINE),'--model',baseline_name], REPORT/'build_baseline.txt'):
        raise RuntimeError('Baseline build failed')
    if not command(['scripts/build_candidate.py','--config',str(CANDIDATE),'--model',MODEL], REPORT/'build_candidate.txt'):
        raise RuntimeError('Candidate build failed')
    # sweep takes an explicit baseline name; labels remain stable in exported comparisons.
    swept, altitude = sweep(targets, config['cruise']['reference_rho_kg_m3'])
    nominal = swept[(swept.model == MODEL) & (swept.tas == 23) & swept.reference_supported]
    rms = float(np.sqrt(np.mean(nominal.error_pct**2)))
    maximum = float(nominal.error_pct.abs().max())
    fit_ok = bool(rms <= 3 and maximum <= 5 and swept.converged.all())
    logs = compare_logs(config, baseline)
    run_name = 'cruise_fleet_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    run_dir = ROOT/'runtime'/run_name
    print(f'Cruise fit: RMS {rms:.3f}%, max {maximum:.3f}%; starting fresh SITL {run_name}', flush=True)
    if args.run_dir:
        run_dir = args.run_dir.resolve()
        summary = json.loads((run_dir/'summary.json').read_text())
        for relative in [f'{MODEL}.xml', 'Engine/empirical_power.xml', 'Engine/Tiguar_20x10_CCW.xml']:
            if (run_dir/'aircraft'/MODEL/relative).read_bytes() != (ROOT/'aircraft'/MODEL/relative).read_bytes():
                raise ValueError(f'Run model does not match candidate: {relative}')
        if (run_dir/'sitl.parm').read_bytes() != (ROOT/'config/sitl.parm').read_bytes():
            raise ValueError('Run parameters do not match current parameters')
        sitl_ok = bool(summary.get('valid') and summary.get('profile') and summary.get('model') == MODEL)
        (REPORT/'sitl_run.txt').write_text(f'Reused matching completed run: {run_dir}\n'+json.dumps(summary, indent=2))
    else:
        sitl_ok = command(['scripts/run_sitl.py','--model',MODEL,'--name',run_name,'--profile','--duration','620'], REPORT/'sitl_run.txt')
    checks_ok = command(['scripts/check_pipeline.py','--model',MODEL,'--config',str(CANDIDATE),'--run-dir',str(run_dir)], REPORT/'pipeline_checks.txt') if sitl_ok else False
    accepted = fit_ok and sitl_ok and checks_ok
    validation = dict(rms_error_pct=rms, max_error_pct=maximum, all_sweep_points_converged=bool(swept.converged.all()),
                      fit_pass=fit_ok, sitl_completed=sitl_ok, pipeline_checks_pass=checks_ok,
                      run_dir=str(run_dir.relative_to(ROOT)), reference_density_altitude_ft=altitude,
                      decision='Accepted: fleet cruise model is the default.' if accepted else 'Not accepted: existing default retained; candidate artifacts available for review.')
    config['cruise']['status'] = 'accepted' if accepted else 'rejected'
    config['cruise']['validation'] = dict(validation)
    CANDIDATE.write_text(json.dumps(config, indent=2)+'\n')
    if accepted:
        if not command(['scripts/build_candidate.py','--config',str(CANDIDATE)], REPORT/'build_default.txt'):
            shutil.copytree(backup, baseline_model, dirs_exist_ok=True)
            raise RuntimeError('Default build failed; aircraft restored')
        shutil.copy2(CANDIDATE, ROOT/'config/calibration.json')
        manifest_path = ROOT/'config/validated_artifacts.json'
        manifest = json.loads(manifest_path.read_text())
        for relative in list(manifest) + ['rpm_cruise_by_aircraft.csv', 'config/cruise_calibration.json']:
            manifest[relative] = hashlib.sha256((ROOT/relative).read_bytes()).hexdigest()
        manifest_path.write_text(json.dumps(manifest, indent=2)+'\n')
    report(config, baseline, targets, swept, logs, validation, run_dir)
    print(json.dumps(validation, indent=2))
    print(f'Report: {REPORT / "index.html"}')


if __name__ == '__main__':
    main()
