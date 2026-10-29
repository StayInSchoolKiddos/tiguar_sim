#!/usr/bin/env python3
"""Build a standalone HTML validation report from recorded evidence."""
import base64
import html
import json
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1]

def image(path):
    return '<img src="data:image/png;base64,'+base64.b64encode(path.read_bytes()).decode()+'">'

def table(df):return df.to_html(index=False,float_format=lambda x:f'{x:.3f}',border=0,escape=True)

def main():
    cfg=json.loads((ROOT/'config/calibration.json').read_text())
    assumptions=json.loads((ROOT/'config/model_assumptions.json').read_text())
    real=pd.read_csv(ROOT/'analysis/2026-05-01/segments.csv')
    baseline=pd.read_csv(ROOT/'runtime/baseline_clean/telemetry.csv')
    sim=pd.read_csv(ROOT/'runtime/candidate_final_scaled/telemetry.csv')
    truth=pd.read_csv(ROOT/'runtime/candidate_final_scaled/jsbsim-state.csv')
    truth.columns=[c.replace('/fdm/jsbsim/','') for c in truth.columns]
    t0=sim[sim.armed==1].t.min()
    labels={3:'Right loiter 23 m/s',5:'Right loiter 25 m/s',7:'Climb to 130 m',8:'Descend to 100 m',9:'Left loiter 23 m/s',10:'Continued left loiter'}
    phases=[]
    for seq,label in labels.items():
        w=sim[sim.mission_seq==seq];end=w[w.t>w.t.max()-20]
        target_alt=130 if seq==7 else 100;target_eas=25 if seq==5 else 23
        phases.append({'phase':label,'mean_eas_m_s':end.eas.mean(),'eas_std':end.eas.std(),'mean_alt_m':end.alt.mean(),
                       'alt_std':end.alt.std(),'bank_deg':end.roll.mean(),'throttle_pct':end.throttle_pct.mean(),
                       'settled_check':bool(abs(end.eas.mean()-target_eas)<1 and abs(end.alt.mean()-target_alt)<3)})
    phases=pd.DataFrame(phases);phases.to_csv(ROOT/'analysis/sitl_phases.csv',index=False)
    rt=real[real.kind=='turn']
    comparison=[{'source':'May 1 real turns (9 training windows)','eas_m_s':rt.eas.mean(),'bank_deg':rt.roll.mean(),
                 'throttle_pct':100*rt.throttle_output_norm.mean(),'rpm':rt.rpm.mean(),'climb_m_s':rt.climb.mean(),'turn_deg_s':rt.turn_rate.mean()}]
    for label,w,jsb in [('Original SITL',baseline[baseline.t>baseline.t.max()-60],None),
                        ('Candidate SITL',sim[sim.mission_seq==3],truth)]:
        w=w[w.t>w.t.max()-40]
        rate=np.rad2deg((w.q*np.sin(np.deg2rad(w.roll))+w.r*np.cos(np.deg2rad(w.roll)))/np.cos(np.deg2rad(w.pitch)))
        rpm=np.nan
        if jsb is not None:
            v=jsb[jsb.Time.between(w.t.min(),w.t.max())];rpm=v['propulsion/engine/propeller-rpm'].mean()
        comparison.append({'source':label,'eas_m_s':w.eas.mean(),'bank_deg':w.roll.mean(),'throttle_pct':w.throttle_pct.mean(),
                           'rpm':rpm,'climb_m_s':w.climb.mean(),'turn_deg_s':rate.mean()})
    comparison=pd.DataFrame(comparison);comparison.to_csv(ROOT/'analysis/loiter_comparison.csv',index=False)
    fig,axes=plt.subplots(4,1,figsize=(12,11),sharex=True)
    axes[0].plot(sim.t-t0,sim.eas,label='Logged airspeed');axes[0].set_ylabel('Airspeed (m/s)')
    axes[1].plot(sim.t-t0,sim.alt);axes[1].set_ylabel('Relative altitude (m)')
    axes[2].plot(sim.t-t0,sim.roll);axes[2].set_ylabel('Bank (deg)')
    axes[3].plot(sim.t-t0,sim.throttle_pct);axes[3].set_ylabel('Throttle (%)');axes[3].set_xlabel('Seconds since arming')
    for seq,label in labels.items():
        w=sim[sim.mission_seq==seq]
        for ax in axes:ax.axvline(w.t.min()-t0,color='gray',alpha=.35);ax.grid(alpha=.2)
        axes[0].text(w.t.min()-t0,32,label,rotation=90,fontsize=8,va='top')
    axes[0].set_ylim(0,34)
    fig.suptitle('ArduPlane 4.5.6 + JSBSim: candidate verification mission (calm simulated atmosphere)')
    fig.tight_layout();fig.savefig(ROOT/'reports/sitl_profile.png',dpi=140);plt.close(fig)
    # Per-flight overview is evidence of the extracted source data, not a simulator overlay.
    fig,axes=plt.subplots(3,1,figsize=(12,8),sharex=True)
    for date,color in [('2026-05-01','tab:blue'),('2026-04-30','tab:orange')]:
        d=pd.read_csv(ROOT/f'analysis/{date}/samples.csv.gz').iloc[::10]
        t=(d.t-d.t.min())/60
        for ax,col in zip(axes,['eas','altitude','throttle_output_norm']):ax.plot(t,d[col],label=date,color=color,lw=.7);ax.grid(alpha=.2)
    for ax,label in zip(axes,['Airspeed (m/s)','EKF relative altitude (m)','Normalized throttle']):ax.set_ylabel(label)
    axes[0].legend();axes[-1].set_xlabel('Minutes since start of each BIN');fig.tight_layout()
    fig.savefig(ROOT/'reports/log_overview.png',dpi=140);plt.close(fig)
    sweep=pd.read_csv(ROOT/'analysis/jsbsim_trim_sweep.csv')
    sweep=sweep[(sweep.model=='tig_calibrated')&(sweep.climb_requested==0)&(sweep.bank_requested==0)]
    scores=pd.DataFrame([{'dataset':name,**values} for name,values in cfg['scores'].items()])
    dyn=json.loads((ROOT/'analysis/dynamics_diagnostic.json').read_text())
    pass_all=phases.settled_check.all()
    status='PASS' if pass_all else 'FAIL'
    checks=(ROOT/'runtime/checks.log').read_text() if (ROOT/'runtime/checks.log').exists() else 'Not run'
    doc=f'''<!doctype html><html lang="en"><meta charset="utf-8"><title>Tiguar simulation validation</title>
<style>body{{font:16px/1.55 system-ui,sans-serif;max-width:1150px;margin:40px auto;padding:0 24px;color:#182b3a}}h1,h2{{line-height:1.2}}table{{border-collapse:collapse;width:100%;font-size:14px}}th,td{{padding:9px;border-bottom:1px solid #d6e0e5;text-align:right}}th:first-child,td:first-child{{text-align:left}}img{{max-width:100%;height:auto}}.notice{{background:#fff1cd;padding:18px;border-left:5px solid #c58b00}}pre{{white-space:pre-wrap;background:#edf3f6;padding:15px;font-size:13px}}code{{background:#edf3f6}}</style>
<h1>Tiguar simulation validation</h1><p>Prepared October 28, 2026. Original aircraft preserved; candidate: <code>aircraft/tig_calibrated</code>.</p>
<p><a href="trajectories/index.html">Real vs simulated state/command trajectories and delta plots</a> · <a href="trajectories/tiguar_real_vs_sim.pdf">Download comparison PDF</a></p>
<div class="notice"><strong>Working simulator; limited flight-data validation.</strong> Coupled mission settled checks: <b>{status}</b>.
The candidate matches the May 1 cruise/loiter operating region, but independent April 30 climb prediction misses the proposed 0.5 m/s target.
Do not interpret successful setpoint tracking as a validated thrust curve or complete aerodynamic model.</div>
<h2>Evidence and data separation</h2><p>Two BINs yield 3 flights on April 30 and 13 on May 1. Selection retained 8 and 35 nonoverlapping ten-second, flaps-up windows, respectively.
After rejecting advance ratios outside the supplied propeller table, 7 held-out and 31 training windows remain for the propulsion/energy comparison.
The training windows span only {cfg['scores']['training']['airspeed_min']:.2f}–{cfg['scores']['training']['airspeed_max']:.2f} m/s EAS. These data cannot identify a broad airspeed/throttle map or a full drag polar.</p>
<p>Inputs: both supplied PDFs, <code>Moments of Inertia.docx</code>, both BINs, and the user's approximate 55 lb takeoff mass including landing gear.
ArduPlane is pinned to <code>bbf0b066e4baa04b5a746f22de642a25905cd52d</code>; JSBSim is 1.3.3. Real throttle is <code>RCO2.C16</code> with time-dependent servo parameters applied. Simulation outputs are explicitly remapped to equivalent axes 1–4.</p>
{image(ROOT/'reports/log_overview.png')}
<h2>Throttle, RPM, airspeed, and altitude response</h2><p>Fit: throttle → equivalent shaft power → propeller RPM/thrust. Drag is constrained through the energy balance using logged TAS, climb and acceleration.
Only effective thrust scale and zero-lift drag are fitted jointly; induced drag k=0.04 is fixed. Thrust is inferred, not measured. The fitted zero-lift drag coefficient is {cfg['cd0']:.5f}, and effective thrust multiplier is {cfg['thrust_scale']:.3f}.</p>
{table(scores)}
<p><code>climb_rmse_from_rpm</code> uses measured RPM; <code>climb_rmse_from_throttle</code> also tests the fitted throttle-to-power relationship. These are steady-window energy-model predictions, not full time-domain JSBSim replays. The independent-day discrepancy remains unresolved; mass/fuel/CG, propulsion configuration, wind and sensor calibration are possible contributors.</p>
{image(ROOT/'reports/performance_fit.png')}
<h2>Actual ArduPilot–JSBSim flight results</h2><p>The same logged roll/pitch/TECS gains were used in the original and candidate simulations. The candidate completed takeoff, right loiter, a 23→25→23 m/s speed change, 100→130→100 m altitude changes, and left loiter. Calm simulated air does not reproduce the real flight's wind/gust variability.</p>
{table(phases)}{image(ROOT/'reports/sitl_profile.png')}
<h2>Loiter comparison at approximately 23 m/s</h2><p>Real values average nine May 1 windows; simulator values use settled right-turn portions. These are matched operating conditions, not a replay of the same route and weather. Candidate RPM is taken from JSBSim truth; baseline RPM was not recorded in the telemetry used here.</p>
{table(comparison)}
<p>There are no qualifying held-out April 30 turn windows. Consequently, this is training-day agreement, not independent validation of banking dynamics. Transient roll/pitch derivative regression was weak: pitch held-out error was {dyn['pitch']['held_out_rmse']:.5f}, versus {dyn['pitch']['held_out_zero_prediction_rmse']:.5f} for a zero-acceleration predictor. Those exploratory coefficients were not applied.</p>
<h2>Conditional JSBSim level-flight map</h2><p>These are equilibrium predictions in standard atmosphere at 1,200 ft, not measurements. The 18–28 m/s sweep extends beyond the flight-data support. Temporary equilibrium-only models use increased rotor inertia to stabilize JSBSim's 0.5 s internal engine-trim iteration; coupled runs do not use this change.</p>
{table(sweep[['eas','success','throttle','rpm','alpha_deg','thrust_n']])}
<p>Some low-power descent trims fail with the truncated propeller table. Windmilling and negative-thrust behavior need better propeller data before claiming descent/landing fidelity.</p>
<h2>Adjustments and unresolved assumptions</h2><ul>
<li>Span changed from 4.10 to 3.888 m; provisional trapezoidal area 1.041 m² and mean aerodynamic chord follow the reported root/tip chords. Approximate HQ airfoil labels were not treated as exact coordinate data.</li>
<li>Mass set to approximately 55 lb (24.95 kg), including gear. Actual fuel burn and CG are unknown. Loaded CG is provisionally x=0.94 m, z=0 on the model datum.</li>
<li>Loaded inertias are 20.5/15.5/34 kg·m². Yaw 34 is a prior: the supplied 41.4 cannot coexist with the supplied roll/pitch values for one configuration about a common origin. Fixture corrections and measurement configurations remain uncertain.</li>
<li>Effective roll/pitch/yaw moment-function scales are {assumptions['moment_coefficient_scales']['ROLL']:.3f}/{assumptions['moment_coefficient_scales']['PITCH']:.3f}/{assumptions['moment_coefficient_scales']['YAW']:.3f}. They preserve approximate seed angular response with the larger inertias; they are not independently identified aerodynamic coefficients. Controller gains were kept fixed.</li>
<li>Ground geometry now has tricycle contacts and 0.713 m track; longitudinal positions, height and friction are provisional. Ground handling and landings are not validated.</li>
<li>Signed elevator drag was corrected to an even, nonnegative deflection term; the effective aileron combination no longer cancels opposed surfaces; propeller inertia has explicit units.</li>
<li>The engine is an empirical equivalent shaft-power source, not a detailed DA35/hybrid model. Electric boost usage, actual propeller performance and fuel consumption remain unresolved.</li>
<li>Equivalent control surfaces retain seed travel conventions. Physical ruddervator travel, flap effects, stall/post-stall response and product-of-inertia values need further work.</li></ul>
<h2>Failed experiments retained</h2><p>The first candidate with the seed vertical CG offset pitched up and stalled. An initial baseline attempt encountered a stale JSBSim socket and is explicitly invalid; it is excluded from all comparisons here. The harness now checks the socket and tracks child processes for cleanup. A larger-inertia model without corresponding moment adjustments oscillated and failed altitude/airspeed checks. Its artifacts remain in <code>runtime/candidate_final</code>; the tested revision is <code>runtime/candidate_final_scaled</code>.</p>
<h2>Checks and next evidence needed</h2><pre>{html.escape(checks)}</pre>
<p>The strongest next inputs are measured CG, actual propeller/RPM/thrust data, documented inertia-test setup, and flaps-up maneuvers at several airspeeds with small control and throttle steps. Independent-day climb errors must improve before broad fidelity is claimed. See <code>README.md</code> for run and reproduction commands.</p></html>'''
    (ROOT/'reports/validation.html').write_text(doc)
    print(table(comparison));print('Wrote reports/validation.html; settled checks',status)

if __name__=='__main__':main()
