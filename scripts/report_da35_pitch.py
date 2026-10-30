#!/usr/bin/env python3
"""Report pitch-loop evidence and controlled sensitivity runs without fitting curves."""
import hashlib
import html
import json
import os
from pathlib import Path
import sys
os.environ.setdefault('MPLCONFIGDIR','/tmp/tiguar-mpl')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from pymavlink import DFReader
from extract_logs import extract
from plot_da35_takeoff import prepare
from match_sitl_parameters import snapshot
from investigate_da35_pitch import CASES

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'reports/da35_cruise_analysis'
WORK=ROOT/'runtime/da35_pitch_investigation'
FINAL=ROOT/'runtime/da35_bin_matched_final'


def load(run):
    summary=json.loads((run/'summary.json').read_text());assert summary['valid']
    cache=run/'extracted'
    if not (cache/'samples.csv.gz').exists():extract(run/'logs/00000001.BIN',cache)
    frame,anchor=prepare(pd.read_csv(cache/'samples.csv.gz'))
    parameters=pd.read_csv(cache/'parameters.csv')
    parameters=parameters[parameters.t<=anchor['throttle_onset_s']].groupby('name').tail(1).set_index('name').value
    assert parameters['KFF_THR2PTCH']==0, 'Report assumes no throttle-to-pitch feedforward'
    frame['pitch_target']=frame.pitch_dem+parameters['PTCH_TRIM_DEG']
    truth=pd.read_csv(run/'jsbsim-state.csv');truth.columns=[c.replace('/fdm/jsbsim/','') for c in truth]
    truth['elapsed_s']=truth.Time-anchor['throttle_onset_s']
    reader=DFReader.DFReader_binary(str(run/'logs/00000001.BIN'));rows=[]
    while (m:=reader.recv_match(type='PIDP')) is not None:
        t=m.TimeUS*1e-6-anchor['throttle_onset_s']
        if t>60:break
        if t>=-5:rows.append({'elapsed_s':t,**{k:v for k,v in m.to_dict().items() if k not in ('mavpackettype','TimeUS')}})
    return frame,truth,pd.DataFrame(rows),anchor


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    sources={case:WORK/case for case in CASES};sources['bin_matched']=FINAL
    data={case:load(run) for case,run in sources.items()}
    actual,actual_anchor=prepare(pd.read_csv(ROOT/'analysis/2026-05-01/samples.csv.gz'))
    actual['pitch_target']=actual.pitch_dem+4
    rows=[];provenance={}
    for name,(s,truth,pid,anchor) in data.items():
        w=s[s.elapsed_s.between(0,30)]; idx=w.pitch.idxmax()
        rows.append({'case':name,'peak_pitch_deg_0_30s':w.pitch.max(),'peak_climb_mps_0_30s':w.climb.max(),
                     'peak_pitch_excess_over_target_deg_0_30s':(w.pitch-w.pitch_target).max(),
                     'target_at_peak_deg':w.loc[idx,'pitch_target'],
                     'peak_I_deg_0_6s':pid[pid.elapsed_s.between(0,6)].I.max()})
        s.to_csv(OUT/f'pitch_{name}_samples.csv',index=False)
        provenance[name]={'run':str(sources[name].relative_to(ROOT)),**anchor,
             'model_sha256':hashlib.sha256((sources[name]/'aircraft/tig_da35_cruise/tig_da35_cruise.xml').read_bytes()).hexdigest(),
             'parameters_sha256':hashlib.sha256((sources[name]/'sitl.parm').read_bytes()).hexdigest(),
             'bin_sha256':hashlib.sha256((sources[name]/'logs/00000001.BIN').read_bytes()).hexdigest()}
    metrics=pd.DataFrame(rows);metrics.to_csv(OUT/'pitch_sensitivity.csv',index=False)
    (OUT/'pitch_provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    baseline=data['baseline'][0];matched=data['bin_matched'][0]
    fig,axes=plt.subplots(4,1,figsize=(12,11),sharex=True,layout='constrained')
    for frame,label,color,ls in [(actual,'Actual May 1','#222b36','-'),(baseline,'Original simulation','#e47718','--'),(matched,'BIN-matched simulation','#087e8b','-.')]:
        for ax,col in zip(axes,['pitch','pitch_target','climb','height_m']):
            ax.plot(frame.elapsed_s,frame[col],color=color,ls=ls,label=label,lw=1.8)
    for ax,label in zip(axes,['Pitch attitude (deg)','Pitch demand + trim (deg)','Climb rate (m/s)','Relative height (m)']):
        ax.set_ylabel(label);ax.grid(alpha=.2);ax.set_xlim(-2,40);ax.axvline(0,color='gray',ls=':',lw=1)
    axes[0].legend();axes[-1].set_xlabel('Seconds from sustained throttle onset in each flight')
    fig.suptitle('Same aircraft model and engine: effect of matching flight parameters')
    fig.savefig(OUT/'pitch_bin_matched.png',dpi=150);plt.close(fig)
    s,truth,pid,_=data['baseline']
    fig,axes=plt.subplots(4,1,figsize=(12,11),sharex=True,layout='constrained')
    axes[0].plot(s.elapsed_s,s.pitch,label='Achieved pitch',color='#e47718');axes[0].plot(s.elapsed_s,s.pitch_target,label='Demand including +4° trim',color='#222b36',ls='--')
    axes[0].set_ylabel('Pitch (degrees)')
    for col,label in [('I','Integral'),('P','Proportional'),('FF','Feed-forward'),('D','Derivative')]:axes[1].plot(pid.elapsed_s,pid[col],label=label)
    axes[1].set_ylabel('Controller contributions\n(notional output degrees)')
    axes[2].plot(truth.elapsed_s,-np.rad2deg(truth['fcs/elevator-pos-rad']),label='Nose-up elevator deflection')
    axes[2].set_ylabel('Elevator (degrees)\npositive = nose-up')
    for col,label in [('aero/coefficient/Cmde','Elevator'),('aero/coefficient/Cmalpha','Angle of attack'),('aero/coefficient/Cmq','Pitch-rate damping'),('moments/m-prop-lbsft','Propulsion')]:
        axes[3].plot(truth.elapsed_s,truth[col]*1.355817948,label=label)
    axes[3].set_ylabel('Pitch moment (N m)\npositive = nose-up')
    for ax in axes:ax.set_xlim(0,15);ax.axhline(0,color='gray',lw=.6);ax.grid(alpha=.2);ax.legend(fontsize=9)
    axes[-1].set_xlabel('Seconds from sustained throttle onset')
    fig.suptitle('Original simulation: why pitch overshoots')
    fig.savefig(OUT/'pitch_loop_diagnosis.png',dpi=150);plt.close(fig)
    audit=json.loads((ROOT/'config/sitl.audit.json').read_text())
    loaded,_=snapshot(FINAL/'logs/00000001.BIN',data['bin_matched'][3]['throttle_onset_s'])
    checks=[]
    for r in audit['parameters']:
        if r['action']=='not imported':continue
        expected=r['simulator_value'];got=loaded.get(r['name'])
        checks.append({**r,'loaded_value':got,'matches':got is not None and np.isclose(got,expected,rtol=1e-6,atol=1e-7)})
    checks=pd.DataFrame(checks);checks.to_csv(OUT/'bin_parameter_verification.csv',index=False)
    if not checks.matches.all():raise ValueError(checks[~checks.matches].to_string(index=False))
    current=metrics.set_index('case');b=current.loc['baseline'];m=current.loc['bin_matched']
    profile_checks=pd.read_json(OUT/'parameter_profile_validation.json')
    profile_table=profile_checks[['model','parameter_values_verified','all_parameters_match','pipeline_passed']].to_html(index=False,border=0)
    table=metrics.rename(columns={'case':'Experiment','peak_pitch_deg_0_30s':'Peak pitch (deg)',
        'peak_climb_mps_0_30s':'Peak climb (m/s)','peak_pitch_excess_over_target_deg_0_30s':'Max pitch above demand (deg)',
        'target_at_peak_deg':'Demand at peak (deg)','peak_I_deg_0_6s':'Peak integral, first 6 s'})
    text=f'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>Tiguar pitch investigation and BIN parameter matching</title>
<style>body{{font:17px/1.6 system-ui;max-width:1100px;margin:32px auto;padding:0 20px;color:#202b38}}img{{width:100%}}table{{border-collapse:collapse;width:100%}}td,th{{padding:8px;border-bottom:1px solid #ccc;text-align:left!important}}.scroll{{overflow-x:auto}}.note{{background:#fff4df;padding:16px}}details{{margin:20px 0}}pre{{white-space:pre-wrap}}</style>
<p><a href="index.html">Back to cruise and takeoff comparisons</a></p><h1>Pitch overshoot and BIN-matched parameters</h1>
<p>This investigation records the earlier BIN-matched configuration. The cruise-tuned model now adds an approved 75% takeoff throttle limit; see <a href="default75.html">the current-default and steady-cruise checks</a>.</p>
{'<p><a href="climb_options.html">See the tested options for reducing climb rate</a></p>' if (OUT/'climb_options.html').exists() else ''}
<p>The original simulation builds substantial nose-up integral output while still on the ground. At about 6 seconds it is already above its pitch demand, but the accumulated integral term still commands nose-up elevator. This is a control-loop and ground-to-air transition problem; it is not evidence of a reversed elevator or a direct engine pitching moment.</p>
<p>Matching the complete selected flight-parameter snapshot changes peak pitch from <strong>{b.peak_pitch_deg_0_30s:.1f}° to {m.peak_pitch_deg_0_30s:.1f}°</strong> in the first 30 seconds. Engine, aerodynamic coefficients, inertia and mission are identical for these two simulated runs. The actual May 1 takeoff peaks at {actual[actual.elapsed_s.between(0,30)].pitch.max():.1f}° during the same event-relative interval.</p>
<img src="pitch_bin_matched.png" alt="Actual, original simulation and BIN-matched simulation: pitch, demand, climb and height.">
<p>Demand includes the recorded +4° pitch trim. The navigation pitch alone understates the attitude demand. All curves use independently detected throttle onset, not fitted time offsets. The actual mission, wind, loading, flap aerodynamics and electric assistance are not replayed.</p>
<h2>What the controlled experiments establish</h2>
<div class="scroll">{table.to_html(index=False,float_format=lambda x:f'{x:.2f}',border=0)}</div>
<p><code>scaling_speed</code> changes only SCALING_SPEED from 15 to 23; <code>rotation_speed</code> changes only TKOFF_ROTATE_SPD from 0 to 17 m/s; <code>both_settings</code> applies both. The <code>unscaled_moments</code> cases remove the provisional 3.78× multiplier from all five pitch-moment terms while preserving inertia. This also reduces aerodynamic damping and control effectiveness. It makes the overshoot worse, so reducing that multiplier is not a demonstrated fix. It does not validate the individual derivatives.</p>
<p>Each sensitivity run is a single deterministic 75-second mission. These experiments diagnose mechanisms, not statistical uncertainty or flight-model fidelity. Full BIN matching changes multiple parameters and cannot attribute its result to a single setting.</p>
<h2>Pitch loop and moment evidence</h2><img src="pitch_loop_diagnosis.png" alt="Original simulated pitch versus demand, PID contributions, elevator deflection and pitch moments.">
<p>At about 5 seconds, the integral contribution reaches roughly 30 notional output degrees while the aircraft is still near level and the elevator is close to its nose-up travel limit. At 6 seconds, pitch is about 29° against roughly 20° demand including trim, yet elevator remains nose-up. The propulsive pitching moment in this interval is tiny compared with the elevator moment. Static stability, damping, control effectiveness, CG, pitch inertia and ground-contact geometry remain provisional and require separate identification.</p>
<h2>Shared simulator parameters</h2><p><code>config/sitl.parm</code> is generated directly from <code>{html.escape(audit['source'])}</code>, using the latest PARM value at or before {audit['at_time_s']:.0f} seconds (the first May 1 takeoff). It supplies all Tiguar profiles launched by <code>run_tiguar.sh</code> or the default <code>run_sitl.py</code> path.</p>
<p>{audit['matched_count']} flight-control, navigation, takeoff, landing, mode and filter parameters match the recorded snapshot; {audit['override_count']} explicit simulator settings retain equivalent output axes, local operation, logging and environment. All {len(checks)} requested values were verified in the new simulated BIN. The source snapshot differs from end-of-day parameters; later changes are not replayed.</p>
<div class="scroll">{profile_table}</div><p>Each listed profile completed a 620-second mission and the four existing pipeline checks. These checks establish configuration loading and mission tracking, not agreement with actual flight dynamics. <a href="parameter_profile_validation.csv">Per-model loaded parameter verification</a>.</p>
<p class="note">A BIN file contains autopilot configuration, not measured aerodynamic coefficients or physical control-surface calibration. Hardware device IDs, sensor offsets, RC interfaces and real ruddervator output assignments are not copied onto the equivalent simulator axes. Copied flap commands do not add flap aerodynamics, and the recalled electric assistance is still absent.</p>
<details><summary>Imported settings and simulator exceptions</summary><div class="scroll">{checks[['name','source_value','simulator_value','loaded_value','action','matches']].to_html(index=False,border=0)}</div></details>
<p>Recreate the shared configuration: <code>python3 scripts/match_sitl_parameters.py</code>. Run a chosen model with <code>python3 scripts/run_sitl.py --model tig_da35_cruise --name my_run --profile --duration 620</code>. A deliberate experimental override is possible with <code>--param-file</code>.</p>
<p><a href="pitch_sensitivity.csv">Experiment metrics</a> · <a href="pitch_provenance.json">Run provenance</a> · <a href="bin_parameter_verification.csv">Loaded parameter checks</a> · <a href="../../config/sitl.audit.json">Complete import audit, including excluded parameters</a></p></html>'''
    (OUT/'pitch_investigation.html').write_text(text)
    print(metrics.to_string(index=False));print(f'Verified {len(checks)} loaded parameters. Report: {OUT}/pitch_investigation.html')

if __name__=='__main__':main()
