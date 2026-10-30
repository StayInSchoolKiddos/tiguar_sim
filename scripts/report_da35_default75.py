#!/usr/bin/env python3
"""Check the promoted takeoff default, cruise tracking and measured level-flight RPM."""
import hashlib
import json
import os
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR','/tmp/tiguar-mpl')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from extract_logs import extract
from plot_da35_takeoff import prepare
from sitl_parameters import default_text,values
from match_sitl_parameters import snapshot

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'reports/da35_cruise_analysis'
FULL=ROOT/'runtime/da35_default75_validation'
STRAIGHT=ROOT/'runtime/da35_default75_straight'
BASE=ROOT/'runtime/da35_bin_matched_final'
MODEL='tig_da35_cruise'

def load(run,check_current=True):
    summary=json.loads((run/'summary.json').read_text());assert summary['valid'] and summary['model']==MODEL
    if check_current:assert (run/'sitl.parm').read_text()==default_text(MODEL)
    for rel in [f'{MODEL}.xml','Engine/DA35.xml','Engine/Tiguar_20x10_CCW.xml']:
        assert (run/'aircraft'/MODEL/rel).read_bytes()==(ROOT/'aircraft'/MODEL/rel).read_bytes()
    cache=run/'extracted';source=hashlib.sha256((run/'logs/00000001.BIN').read_bytes()).hexdigest()
    if not (cache/'source.sha256').exists() or (cache/'source.sha256').read_text()!=source:
        extract(run/'logs/00000001.BIN',cache);(cache/'source.sha256').write_text(source)
    frame=pd.read_csv(cache/'samples.csv.gz')
    truth=pd.read_csv(run/'jsbsim-state.csv')[['Time','/fdm/jsbsim/propulsion/engine/propeller-rpm']]
    truth.columns=['truth_time','truth_rpm']
    frame=pd.merge_asof(frame.sort_values('t'),truth.sort_values('truth_time'),left_on='t',right_on='truth_time',direction='nearest',tolerance=.03)
    frame['rpm']=frame.truth_rpm
    if check_current:
        loaded,_=snapshot(run/'logs/00000001.BIN',60)
        assert all(k in loaded and np.isclose(loaded[k],v,rtol=1e-6,atol=1e-7) for k,v in values(default_text(MODEL)).items())
    return frame

def main():
    full=load(FULL);straight=load(STRAIGHT);baseline=load(BASE,False)
    assert full[full.stage==1].throttle_output_norm.max()<=.751
    takeoff,anchor=prepare(full);actual,_=prepare(pd.read_csv(ROOT/'analysis/2026-05-01/samples.csv.gz'))
    rows=[]
    for label,s in [('Actual May 1',actual),('New default: 75%',takeoff),('Previous default',prepare(baseline)[0])]:
        w=s[s.elapsed_s.between(0,60)];band=w[(w.stage==1)&w.height_m.between(10,80)];cross=w[w.height_m>=50]
        rows.append({'Flight':label,'Peak climb, first 30 s (m/s)':w[w.elapsed_s<=30].climb.max(),
            'Mean takeoff climb at 10–80 m (m/s)':band.climb.mean(),'Seconds to +50 m':cross.elapsed_s.iloc[0] if len(cross) else np.nan,
            'Peak RPM, first 60 s':w.rpm.max()})
    takeoff_metrics=pd.DataFrame(rows);takeoff_metrics.to_csv(OUT/'default75_takeoff.csv',index=False)
    phases=[]
    for label,run in [('Previous default',BASE),('New default: 75%',FULL)]:
        t=pd.read_csv(run/'telemetry.csv')
        for seq,alt,speed in [(3,100,23),(5,100,25),(7,130,23),(8,100,23),(9,100,23)]:
            w=t[t.mission_seq==seq];w=w[w.t>w.t.max()-20]
            assert len(w)>=190
            phases.append({'Configuration':label,'Phase':seq,'Target EAS (m/s)':speed,'Mean EAS (m/s)':w.eas.mean(),
                'EAS RMSE (m/s)':np.sqrt(np.mean((w.eas-speed)**2)), 'Target height (m)':alt,
                'Mean height (m)':w.alt.mean(),'Height RMSE (m)':np.sqrt(np.mean((w.alt-alt)**2)),
                'Mean throttle (%)':w.throttle_pct.mean(),'Mean climb (m/s)':w.climb.mean()})
    phases=pd.DataFrame(phases);phases.to_csv(OUT/'default75_cruise_phases.csv',index=False)
    # Last 60 seconds of a dedicated 6 km straight leg, after takeoff and settling.
    w=straight[straight.t>straight.t.max()-60].copy();w['elapsed']=w.t-w.t.min()
    t=pd.read_csv(STRAIGHT/'telemetry.csv');t=t[t.t>t.t.max()-60]
    assert (t.mission_seq==2).all() and (w.stage==3).all()
    assert w[['eas','altitude','climb','roll','rpm','throttle_output_norm']].notna().all().all()
    # Use autopilot relative altitude for target comparison, identical to the flight checks.
    stats={'run':str(STRAIGHT.relative_to(ROOT)),'duration_s':float(w.t.max()-w.t.min()),
        'mean_eas_mps':float(w.eas.mean()),'eas_rmse_mps':float(np.sqrt(np.mean((w.eas-23)**2))),
        'mean_height_m':float(t.alt.mean()),'height_rmse_m':float(np.sqrt(np.mean((t.alt-100)**2))),
        'mean_climb_mps':float(w.climb.mean()),'climb_rms_mps':float(np.sqrt(np.mean(w.climb**2))),
        'mean_bank_deg':float(w.roll.mean()),'bank_rms_deg':float(np.sqrt(np.mean(w.roll**2))),
        'mean_rpm':float(w.rpm.mean()),'rpm_std':float(w.rpm.std()),
        'mean_throttle_pct':float(w.throttle_output_norm.mean()*100)}
    stats['tracking_pass']=bool(stats['eas_rmse_mps']<1 and stats['height_rmse_m']<3 and stats['climb_rms_mps']<.2 and stats['bank_rms_deg']<2)
    (OUT/'default75_straight_checks.json').write_text(json.dumps(stats,indent=2)+'\n')
    w.to_csv(OUT/'default75_straight_samples.csv',index=False)
    fig,axes=plt.subplots(3,2,figsize=(12,9),sharex=True,layout='constrained')
    for ax,col,label,target in zip(axes.flat,['eas','altitude','climb','roll','rpm','throttle_output_norm'],
        ['EAS (m/s)','Height relative to home (m)','Climb (m/s)','Bank angle (deg)','Propeller RPM','Throttle (%)'],[23,100,0,0,None,None]):
        x=t.t-w.t.min() if col=='altitude' else w.elapsed
        y=t.alt if col=='altitude' else w[col]*(100 if col=='throttle_output_norm' else 1)
        ax.plot(x,y,color='#e47718',label='Simulated')
        if target is not None:ax.axhline(target,color='#777777',ls='--',label='Target')
        ax.set_ylabel(label);ax.grid(alpha=.2);ax.legend(fontsize=8)
    for ax,limits in zip(axes.flat,[(22.5,23.5),(99.5,100.5),(-.2,.2),(-2,2),(4500,5200),(20,30)]):ax.set_ylim(*limits)
    for ax in axes[-1]:ax.set_xlabel('Seconds within final steady 60-second window')
    fig.suptitle('New default: steady, straight cruise at 23 m/s and 100 m')
    fig.savefig(OUT/'default75_straight.png',dpi=150);plt.close(fig)
    windows=pd.read_csv(OUT/'cruise_windows.csv');pred=pd.read_csv(OUT/'predictions.csv')
    level=windows[windows.kind=='level'][['date','flight','start_s']]
    level=pred[pred.model==MODEL].merge(level,on=['date','flight','start_s']).sort_values(['date','start_s']).reset_index(drop=True)
    level.to_csv(OUT/'default75_level_rpm.csv',index=False)
    scores=[]
    for day,g in level.groupby('date'):
        scores.append({'Day':day,'Samples':len(g),'Partition':'Fit' if day=='2026-05-01' else 'Held out',
            'RPM RMSE':float(np.sqrt(np.mean(g.rpm_error**2))),'Mean absolute RPM error (%)':float(g.relative_error_pct.abs().mean())})
    scores=pd.DataFrame(scores);scores.to_csv(OUT/'default75_level_rpm_metrics.csv',index=False)
    fig,ax=plt.subplots(figsize=(12,4),layout='constrained');x=np.arange(1,len(level)+1)
    ax.scatter(x,level.rpm_observed,color='#222b36',label='Actual level-flight RPM',s=55)
    ax.scatter(x,level.rpm_predicted,color='#e47718',marker='D',label='Steady model at recorded throttle / TAS / density',s=40)
    ax.set_xticks(x,[f'{r.date[5:]}\n{r.start_s:.0f}s' for r in level.itertuples()]);ax.set_ylabel('RPM');ax.grid(axis='y',alpha=.2);ax.legend()
    ax.set_title('Actual vs model: separate 10-second straight-and-level samples')
    fig.savefig(OUT/'default75_level_rpm.png',dpi=150);plt.close(fig)
    provenance={label:{'run':str(run.relative_to(ROOT)),'parameters_sha256':hashlib.sha256((run/'sitl.parm').read_bytes()).hexdigest(),
        'bin_sha256':hashlib.sha256((run/'logs/00000001.BIN').read_bytes()).hexdigest()}
        for label,run in [('full_mission',FULL),('straight_cruise',STRAIGHT),('previous_default',BASE)]}
    (OUT/'default75_provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    current=phases[phases.Configuration=='New default: 75%']
    fragment=f'''<section id="current-default"><h2>Current default: 75% takeoff throttle, with cruise checks</h2>
<p><code>tig_da35_cruise</code> now applies <code>TKOFF_THR_MAX=75</code> by default through <code>config/models/tig_da35_cruise.parm</code>. The shared BIN snapshot and the other model defaults retain their original takeoff setting. Explicit experimental parameter files override model defaults.</p>
<p><strong>Takeoff is closer, but it does not reproduce the black actual-flight curve.</strong> The height, speed, RPM and command histories still differ. The actual aircraft used up to 100% gasoline throttle plus recalled electric assistance; the simulator uses a 75% cap as an empirical scenario adjustment.</p>
<div class="table-wrap">{takeoff_metrics.to_html(index=False,float_format=lambda v:f'{v:.2f}',border=0)}</div>
<p>Climb means use takeoff-stage samples between 10 and 80 m height, with different time windows for each flight. Time to +50 m is measured from throttle onset. No curves were shifted or scaled to improve agreement.</p>
<h3>Does steady cruise still track?</h3><p>Yes within the checked mission. Across five settled level-turn phases at 23/25 m/s and 100/130 m, the largest EAS RMSE is {current['EAS RMSE (m/s)'].max():.3f} m/s and largest height RMSE is {current['Height RMSE (m)'].max():.3f} m. Values use the final 20 seconds of each phase.</p>
<p>The separate straight-level run checks the final 60 seconds of a 6 km waypoint leg: mean EAS <strong>{stats['mean_eas_mps']:.3f} m/s</strong>, mean relative height <strong>{stats['mean_height_m']:.3f} m</strong>, climb RMS <strong>{stats['climb_rms_mps']:.3f} m/s</strong>, bank RMS {stats['bank_rms_deg']:.3f}°, RPM {stats['mean_rpm']:.0f} ± {stats['rpm_std']:.0f}, throttle {stats['mean_throttle_pct']:.1f}%. This tests simulated tracking, not a replay of the actual flight.</p>
<figure><img src="default75_straight.png" alt="Steady straight cruise: simulated speed, height, climb, bank, RPM and throttle."></figure>
<h3>Does the engine still match recorded level-flight RPM?</h3><p>The engine and its cruise calibration are unchanged. Below are the existing steady-model predictions restricted to the actual straight-and-level windows (turns excluded), evaluated at each window's recorded throttle, TAS and density. May 1 was used for fitting; April 30 is held out. These are cached engine calculations, separate from the coupled mission above.</p>
<div class="table-wrap">{scores.to_html(index=False,float_format=lambda v:f'{v:.2f}',border=0)}</div><figure><img src="default75_level_rpm.png" alt="Actual black RPM markers versus orange steady-model predictions for level-flight windows."></figure>
<p>Only two held-out level windows are available. Good RPM agreement does not verify thrust or fuel consumption; the earlier held-out climb prediction error remains about 1.5 m/s RMS.</p>
<details><summary>Before and after: settled cruise tracking by phase</summary><div class="table-wrap">{phases.to_html(index=False,float_format=lambda v:f'{v:.3f}',border=0)}</div></details>
<p><a href="default75_straight_checks.json">Straight-flight checks</a> · <a href="default75_cruise_phases.csv">Cruise tracking metrics</a> · <a href="default75_level_rpm.csv">Actual level-flight RPM comparisons</a> · <a href="default75_provenance.json">Run provenance</a></p>
<p>Interactive launch: <code>./run_tiguar.sh --model tig_da35_cruise</code>. Automated flight: <code>python3 scripts/run_sitl.py --model tig_da35_cruise --name my_default75_run --profile --duration 620</code>. The takeoff limit is applied automatically; no <code>--param-file</code> is required.</p></section>'''
    (OUT/'default75_section.html').write_text(fragment)
    (OUT/'default75.html').write_text('''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>Tiguar default takeoff and steady cruise</title><style>body{font:17px/1.6 system-ui;max-width:1100px;margin:32px auto;padding:0 20px;color:#202b38}img{width:100%}table{border-collapse:collapse;width:100%}td,th{padding:8px;border-bottom:1px solid #ccc;text-align:left!important}.table-wrap{overflow-x:auto}figure{margin:20px 0}</style><a href="index.html">Back to main report</a>'''+fragment+'</html>')
    print(json.dumps(stats,indent=2));print(scores.to_string(index=False));print(takeoff_metrics.to_string(index=False))
    if not stats['tracking_pass']:raise SystemExit('Straight cruise tracking check failed; see report.')

if __name__=='__main__':main()
