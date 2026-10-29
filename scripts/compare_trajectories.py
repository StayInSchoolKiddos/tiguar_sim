#!/usr/bin/env python3
"""Plot maneuver-aligned flight-log states, demands, outputs, paths and deltas.

No time warping or fitted delays. These independent missions are NOT same-input
replays: differences include command, environment, initial-state and model effects.
"""
import argparse
import base64
import html
import json
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'reports/trajectories'
DATA=ROOT/'analysis/trajectory_comparisons'
COLORS=['#1672ad','#e47b22','#8b3291']
GROUPS={
    'states': [('eas','Airspeed EAS (m/s)'),('height_change','Height change (m)'),
        ('climb','Climb rate (m/s)'),('roll','Roll (deg)'),('pitch','Pitch (deg)'),
        ('heading_change','Heading change (deg)'),('rpm','RPM (real sensor / sim truth)')],
    'rates': [('p_deg','Roll rate p (deg/s)'),('q_deg','Pitch rate q (deg/s)'),('r_deg','Yaw rate r (deg/s)'),
        ('roll_error','Roll − demand (deg)'),('pitch_error','Pitch − demand (deg)'),('height_error','Height − demand (m)')],
    'commands': [('speed_dem','TECS TAS demand (m/s)'),('height_demand','Height demand − initial height (m)'),
        ('roll_dem','Roll demand (deg)'),('pitch_dem','Pitch demand (deg)'),
        ('throttle_output_pct','Throttle output (%)'),('aileron_pct','Aileron output (%)'),
        ('elevator_pct','Elevator output (%)'),('rudder_pct','Rudder output (%)')],
}

def load_sim():
    path=ROOT/'analysis/sitl_comparison/samples.csv.gz'
    if not path.exists():
        from extract_logs import extract
        extract(ROOT/'runtime/candidate_final_scaled/logs/00000001.BIN',path.parent)
    s=pd.read_csv(path)
    truth=pd.read_csv(ROOT/'runtime/candidate_final_scaled/jsbsim-state.csv')
    rpm=truth[['Time','/fdm/jsbsim/propulsion/engine/propeller-rpm']].rename(columns={'Time':'t','/fdm/jsbsim/propulsion/engine/propeller-rpm':'rpm_truth'})
    s=pd.merge_asof(s,rpm.sort_values('t').drop_duplicates('t'),on='t',direction='nearest',tolerance=.06)
    s['rpm']=s.rpm_truth
    return s

def default_cases(real,sim):
    # Shared geometric event, independently detected; no optimization against outcome error.
    def climb_anchor(s):
        candidates=s[(s.flight==1)&(s.altitude>=50)&(s.climb>.5)&(s.flap.abs()<1)&(s.eas>18)]
        return float(candidates.iloc[0].t)
    telemetry=pd.read_csv(ROOT/'runtime/candidate_final_scaled/telemetry.csv')
    turns=pd.read_csv(ROOT/'analysis/2026-05-01/segments.csv')
    return [
        dict(id='may01_climb',title='May 1: climb through 50 m',date='2026-05-01',flight=1,
             real_start=climb_anchor(real['2026-05-01']),sim_start=climb_anchor(sim),duration=30,
             alignment='First flaps-up, airspeed >18 m/s, climbing sample at or above 50 m EKF altitude in flight 1, independently in each run.'),
        dict(id='may01_loiter',title='May 1: right loiter',date='2026-05-01',flight=1,
             real_start=float(turns[turns.kind=='turn'].iloc[0].start_s),
             sim_start=round(float(telemetry[telemetry.mission_seq==3].t.max())-40,1),duration=40,
             alignment='First accepted May 1 right-turn window; simulator uses the last 40 seconds of its first right loiter. Start headings and positions differ.'),
        dict(id='may01_descent',title='May 1: descent',date='2026-05-01',flight=1,
             real_start=float(turns[turns.kind=='descent'].iloc[0].start_s),
             sim_start=round(float(telemetry[telemetry.mission_seq==8].t.min()),1),duration=30,
             alignment='First accepted May 1 descent window versus start of the simulated 130→100 m descent. Altitude targets and bank schedules differ.'),
        dict(id='apr30_climb',title='April 30 held-out day: climb through 50 m',date='2026-04-30',flight=1,
             real_start=climb_anchor(real['2026-04-30']),sim_start=climb_anchor(sim),duration=30,
             alignment='Same 50 m climb selection rule, using the April 30 held-out flight day. This does not make the simulator mission an identical-command validation.'),
    ]

def prepare(s,start,duration):
    w=s[s.t.between(start-.055,start+duration+.055)].copy()
    if len(w)<2:raise ValueError('Selected window has insufficient data')
    w['elapsed']=w.t-start
    for c in ['tecs_alt','yaw','pn','pe','altitude','vn','ve']:
        if not np.isfinite(w[c].iloc[0]):raise ValueError(f'Missing initial {c}')
    w['height_change']=w.tecs_alt-w.tecs_alt.iloc[0]
    w['height_demand']=w.alt_dem-w.tecs_alt.iloc[0]
    w['height_error']=w.tecs_alt-w.alt_dem
    w['heading_change']=np.rad2deg(np.unwrap(np.deg2rad(w.yaw)))-w.yaw.iloc[0]
    for c in ['p','q','r']:w[c+'_deg']=np.rad2deg(w[c])
    for c in ['aileron','elevator','rudder']:w[c+'_pct']=w[c]/45
    w['throttle_output_pct']=w.throttle_output_norm*100
    for c in ['roll','pitch']:w[c+'_error']=w[c]-w[c+'_dem']
    # Translate each path to zero and rotate it into its own initial ground-track frame.
    track=np.arctan2(w.ve.iloc[0],w.vn.iloc[0]);dn=w.pn-w.pn.iloc[0];de=w.pe-w.pe.iloc[0]
    w['forward']=dn*np.cos(track)+de*np.sin(track)
    w['right']=-dn*np.sin(track)+de*np.cos(track)
    w['up']=w.altitude-w.altitude.iloc[0]
    return w,dict(actual_start_s=float(w.t.iloc[0]),initial_ekf_alt_m=float(w.altitude.iloc[0]),
        initial_tecs_height_m=float(w.tecs_alt.iloc[0]),initial_heading_deg=float(w.yaw.iloc[0]),
        initial_ground_track_deg=float(np.rad2deg(track)),initial_north_m=float(w.pn.iloc[0]),initial_east_m=float(w.pe.iloc[0]))

def pair(real,sim,case):
    r,ri=prepare(real,case['real_start'],case['duration']);s,si=prepare(sim,case['sim_start'],case['duration'])
    if not (r.flight==case['flight']).all():raise ValueError('Real window crosses flight boundary')
    if not (s.flight>0).all():raise ValueError('Simulation window is not entirely airborne')
    if not ((r.flap.abs()<1).all() and (s.flap.abs()<1).all()):raise ValueError('Window includes flaps; this model does not validate flap behavior')
    grid=pd.DataFrame({'elapsed':np.round(np.arange(0,case['duration'],.1),6)})
    cols=list(dict.fromkeys([c for g in GROUPS.values() for c,label in g]+['forward','right','up']))
    both=grid.copy()
    for name,w in [('real',r),('sim',s)]:
        # Nearest samples within half a sample period; never bridge telemetry gaps.
        x=pd.merge_asof(grid,w[['elapsed','t']+cols].sort_values('elapsed'),on='elapsed',direction='nearest',tolerance=.055)
        for c in ['t']+cols:both[name+'_'+c]=x[c]
    for c in cols:both['delta_'+c]=both['sim_'+c]-both['real_'+c]
    metrics=[]
    for c in cols:
        e=both['delta_'+c].dropna()
        metrics.append(dict(case=case['id'],signal=c,paired_samples=len(e),coverage=len(e)/len(both),
            bias=float(e.mean()),mae=float(e.abs().mean()),rmse=float(np.sqrt((e*e).mean())),peak_abs=float(e.abs().max())))
    case.update(real_reference=ri,sim_reference=si)
    return both,metrics

def plot_group(data,case,group,pdf):
    signals=GROUPS[group];fig,axes=plt.subplots(len(signals),2,figsize=(15,2.25*len(signals)),sharex=True,layout='constrained')
    for (col,label),(overlay,delta) in zip(signals,axes):
        for key,name,color in [('real','Real flight',COLORS[0]),('sim','ArduPilot + JSBSim',COLORS[1])]:
            overlay.plot(data.elapsed,data[key+'_'+col],label=name,color=color,lw=1.1)
        overlay.set_ylabel(label,fontsize=9)
        delta.plot(data.elapsed,data['delta_'+col],color=COLORS[2],lw=1)
        delta.axhline(0,color='black',ls='--',lw=.7)
        err=data['delta_'+col].dropna()
        delta.text(.98,.94,f'RMSE {np.sqrt(np.mean(err**2)):.2f} | bias {err.mean():+.2f}',transform=delta.transAxes,ha='right',va='top',fontsize=9)
        for ax in [overlay,delta]:ax.grid(alpha=.2);ax.set_xlim(0,case['duration'])
    axes[0,0].legend(loc='best',fontsize=9);axes[0,0].set_title('Overlaid trajectories');axes[0,1].set_title('Delta = simulation − real')
    for ax in axes[-1]:ax.set_xlabel('Elapsed seconds from each selected window start')
    fig.suptitle(case['title']+' — '+group+'\nDifferent missions; event/window aligned, without time warping',fontsize=14)
    path=OUT/f"{case['id']}_{group}.png";fig.savefig(path,dpi=130);pdf.savefig(fig);plt.close(fig)
    return path

def plot_path(d,case,pdf):
    fig=plt.figure(figsize=(14,9),layout='constrained');ax=fig.add_subplot(221);ax3=fig.add_subplot(222,projection='3d')
    height=fig.add_subplot(223);delta=fig.add_subplot(224)
    for key,name,color in [('real','Real flight',COLORS[0]),('sim','ArduPilot + JSBSim',COLORS[1])]:
        ax.plot(d[key+'_forward'],d[key+'_right'],label=name,color=color)
        ax3.plot(d[key+'_forward'],d[key+'_right'],d[key+'_up'],label=name,color=color)
        height.plot(d.elapsed,d[key+'_up'],label=name,color=color)
    ax.scatter([0],[0],c='black',marker='x',label='Aligned starts');ax.axis('equal');ax.legend(fontsize=9)
    ax.set(xlabel='Along initial ground track (m)',ylabel='Right of initial ground track (m)',title='Horizontal path')
    ax3.set(xlabel='Along track (m)',ylabel='Right (m)',zlabel='Height change (m)',title='3D path (independent axis scales)')
    height.set(xlabel='Elapsed time (s)',ylabel='EKF height change (m)',title='Vertical trajectory')
    for c in ['forward','right','up']:delta.plot(d.elapsed,d['delta_'+c],label=c)
    delta.axhline(0,color='black',ls='--',lw=.7);delta.legend();delta.set(xlabel='Elapsed time (s)',ylabel='Sim − real displacement (m)',title='Relative path differences')
    for a in [ax,height,delta]:a.grid(alpha=.2)
    fig.suptitle(case['title']+' — spatial trajectories\nEach start translated to zero and rotated to its own initial ground track; not geographic navigation error',fontsize=13)
    path=OUT/f"{case['id']}_path.png";fig.savefig(path,dpi=140);pdf.savefig(fig);plt.close(fig)
    return path

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--config',type=Path);args=ap.parse_args()
    OUT.mkdir(parents=True,exist_ok=True);DATA.mkdir(parents=True,exist_ok=True)
    real={date:pd.read_csv(ROOT/f'analysis/{date}/samples.csv.gz') for date in ['2026-05-01','2026-04-30']};sim=load_sim()
    cases=json.loads(args.config.read_text()) if args.config else default_cases(real,sim)
    all_metrics=[];sections=[]
    with PdfPages(OUT/'tiguar_real_vs_sim.pdf',metadata={'Title':'Tiguar real vs simulated state and command trajectories'}) as pdf:
        for case in cases:
            d,metrics=pair(real[case['date']],sim,case);all_metrics.extend(metrics)
            d.to_csv(DATA/f"{case['id']}.csv",index=False)
            paths=[plot_group(d,case,g,pdf) for g in GROUPS]+[plot_path(d,case,pdf)]
            blocks=[]
            for path in paths:
                encoded=base64.b64encode(path.read_bytes()).decode()
                blocks.append(f'<details open><summary>{html.escape(path.stem)}</summary><p><a href="{path.name}">Download PNG</a></p><img src="data:image/png;base64,{encoded}"></details>')
            table=pd.DataFrame(metrics).query("signal in ['eas','height_change','climb','roll','pitch','throttle_output_pct','rpm']")
            sections.append(f'<section id="{case["id"]}"><h2>{html.escape(case["title"])}</h2><p>{html.escape(case["alignment"])}</p>'
                f'<p>Real log: {case["date"]}, flight {case["flight"]}, t={case["real_start"]:.1f} s. Simulation t={case["sim_start"]:.1f} s. Duration {case["duration"]} s.</p>'
                +table.to_html(index=False,float_format=lambda x:f'{x:.3f}',border=0)+''.join(blocks)+'</section>')
            print(case['id'], 'paired rows',len(d),'minimum signal coverage',min(m['coverage'] for m in metrics),flush=True)
    pd.DataFrame(all_metrics).to_csv(DATA/'metrics.csv',index=False)
    (DATA/'alignment.json').write_text(json.dumps(cases,indent=2))
    nav=' | '.join(f'<a href="#{c["id"]}">{html.escape(c["title"])}</a>' for c in cases)
    page='''<!doctype html><html lang="en"><meta charset="utf-8"><title>Tiguar real vs simulation trajectories</title>
<style>body{font:16px/1.5 system-ui,sans-serif;max-width:1450px;margin:30px auto;padding:0 20px;color:#203040}img{width:100%;height:auto}table{border-collapse:collapse;font-size:14px}td,th{padding:7px;border-bottom:1px solid #ddd}summary{cursor:pointer;font-weight:bold;padding:12px;background:#edf3f7}.note{padding:18px;background:#fff1d2;border-left:5px solid #c88e15}section{margin-top:40px}details{margin:16px 0}a{color:#176caa}</style>
<h1>Real Tiguar vs simulated Tiguar: states, commands, and deltas</h1>
<p><a href="tiguar_real_vs_sim.pdf">Download the complete 16-page PDF</a> · <a href="../../analysis/trajectory_comparisons/metrics.csv">Difference metrics CSV</a> · <a href="../../analysis/trajectory_comparisons/alignment.json">Exact alignment definitions</a></p>
<div class="note"><strong>These are maneuver-aligned comparisons of independently flown missions, not identical-command replays.</strong>
Both trajectories are real recorded data: the aircraft BIN and the completed ArduPilot–JSBSim BIN. Differences include command schedules, initial conditions, wind, sensing, and airframe behavior. Read the command plots beside the state plots; RMSE here is a window difference, not isolated model-prediction error.</div>
<p>Blue = actual flight; orange = simulation; purple = simulation minus actual. Samples are paired at 10 Hz within ±55 ms of each window-relative timestamp. No fitted delay, time stretching, or smoothing is applied. Missing pairs remain gaps.</p>
<p>State height is TECS height relative to each window's initial height; heading is unwrapped change from its initial value. Height demand uses the same initial-height reference, preserving tracking error. TECS speed demand is true airspeed (TAS), whereas the state panel shows equivalent airspeed (EAS); their numerical values should not be compared as a tracking error. Body rates are deg/s. Aileron/elevator/rudder are normalized AETR controller outputs (±100%), not measured surface angles. Throttle is actual normalized servo output (real channel 16, simulated channel 3); raw PWM is not compared across different hardware mappings. RPM is measured on the aircraft and JSBSim truth in the simulation. Other states/demands come from matching ArduPilot log fields in both runs.</p>
<p>Spatial plots use EKF N/E/height, translated to each window's start and rotated into its own initial ground-track frame. They compare relative path shape and displacement, not absolute geographic position. Simulation is calm; recorded flights contain estimated wind. April 30 was held out from propulsion fitting.</p>
<nav>'''+nav+'</nav>'+''.join(sections)+'</html>'
    (OUT/'index.html').write_text(page)
    print('Wrote',OUT/'index.html')

if __name__=='__main__':main()
