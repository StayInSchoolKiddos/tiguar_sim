#!/usr/bin/env python3
"""One-parameter empirical cruise correction, with held-out flight-day analysis."""
import argparse
import hashlib
import html
import json
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
import numpy as np
import pandas as pd
from scipy.optimize import brentq, minimize_scalar
from cruise_calibration import ROOT, bench, settle, RHO_CONVERSION
from calibrate import add_physics, MASS, G, K
from sitl_parameters import default_text
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

MODEL='tig_da35_cruise'
OUT=ROOT/'reports/da35_cruise_analysis'
CONFIG=ROOT/'config/da35_cruise_tuning.json'
TRAIN='2026-05-01'
TEST='2026-04-30'


def effective_throttle(u, scale):
    # Correction is confined to mid-throttle, preserving idle and high-power endpoints.
    weight=np.interp(u,[0,.1,.2,.5,.7,1],[0,0,1,1,0,0])
    return np.asarray(u)*(1+(scale-1)*weight)


def select_cruise():
    selected=[];audit=[]
    for day in [TRAIN,TEST]:
        samples=pd.read_csv(ROOT/f'analysis/{day}/samples.csv.gz')
        starts=samples[samples.flight>0].groupby('flight').t.min()
        segments=add_physics(pd.read_csv(ROOT/f'analysis/{day}/segments.csv'))
        for r in segments.itertuples():
            w=samples[samples.t.between(r.start_s-1e-5,r.end_s+1e-5)]
            reasons=[]
            if r.kind not in ['level','turn']:reasons.append('not level/turn cruise')
            if not r.prop_in_range:reasons.append('measured propeller advance ratio outside table')
            if not 21<=r.eas<=25:reasons.append('outside 21–25 m/s EAS selection')
            if r.start_s-starts.loc[r.flight]<60-1e-5:reasons.append('within first 60 s after flight detection')
            if len(w)<98 or not (w.stage==3).all():reasons.append('not continuously normal flight stage')
            audit.append(dict(date=day,start_s=r.start_s,kind=r.kind,selected=not reasons,reason='; '.join(reasons)))
            if not reasons:
                data=r._asdict();data['date']=day;selected.append(data)
    frame=pd.DataFrame(selected)
    if frame[frame.date==TRAIN].empty or frame[frame.date==TEST].empty:raise ValueError('Missing training or held-out cruise windows')
    pd.DataFrame(audit).to_csv(OUT/'selection_audit.csv',index=False)
    frame.to_csv(OUT/'cruise_windows.csv',index=False)
    return frame


def state(model,speed,altitude):
    f=bench(model,speed,altitude)
    f['propulsion/set-running']=0
    return f


def predict(model,row,throttle):
    f=state(model,row.tas,row.altitude_ft)
    rpm,converged,_=settle(f,float(throttle))
    j=row.tas/(rpm/60*.508) if rpm>0 else float('inf')
    if not converged or not np.isfinite(rpm) or not f['propulsion/engine/set-running']:
        raise RuntimeError('Unsettled/nonrunning engine; cannot use calibration point')
    return rpm,f['propulsion/engine/thrust-lbs']*4.448221615,j


def build(scale):
    source=ROOT/'aircraft/tig_da35';dest=ROOT/'aircraft'/MODEL
    dest.mkdir(parents=True,exist_ok=True)
    shutil.copytree(source/'Engine',dest/'Engine',dirs_exist_ok=True)
    shutil.copy2(source/'reset.xml',dest/'reset.xml')
    tree=ET.parse(source/'tig_da35.xml');root=tree.getroot();root.set('name',MODEL)
    root.find('fileheader/description').text='Empirical DA35 cruise correction fitted to May 1 level/turn windows; April 30 held out. See config/da35_cruise_tuning.json. Not a measured DA35 rating or hybrid takeoff model.'
    table=root.find("flight_control/channel[@name='DA35 carburetor and starting']/fcs_function[@name='fcs/da35-carburetor']/function/table/tableData")
    original=np.array([[float(x) for x in line.split()] for line in table.text.strip().splitlines()])
    grid=np.unique(np.round(np.r_[np.linspace(0,1,1001),original[:,0]],8))
    values=np.interp(effective_throttle(grid,scale),original[:,0],original[:,1])
    if not np.all(np.diff(values)>=-1e-12):raise ValueError('Nonmonotone throttle mapping')
    table.text='\n'+'\n'.join(f'{u:.8f} {v:.10f}' for u,v in zip(grid,values))+'\n'
    ET.indent(tree);tree.write(dest/f'{MODEL}.xml',encoding='utf-8',xml_declaration=True)


def evaluate(windows):
    c=json.loads((ROOT/'config/calibration.json').read_text());rows=[]
    for row in windows.itertuples():
        for model in ['tig_da35',MODEL]:
            rpm,thrust,j=predict(model,row,row.throttle_output_norm)
            drag=row.qS*(c['cd0']+K*row.CL**2)
            climb=(thrust-drag-MASS*row.accel)*row.tas/(MASS*G)
            rows.append(dict(date=row.date,flight=row.flight,start_s=row.start_s,kind=row.kind,
                             model=model,throttle=row.throttle_output_norm,eas=row.eas,tas=row.tas,
                             rpm_observed=row.rpm,rpm_std=row.rpm_std,rpm_predicted=rpm,
                             rpm_error=rpm-row.rpm,relative_error_pct=100*(rpm/row.rpm-1),
                             climb_observed=row.climb,climb_predicted=climb,thrust_n=thrust,
                             advance_ratio=j,prediction_supported=bool(0<=j<=.6674)))
    df=pd.DataFrame(rows)
    # Do not silently remove points on which the new mapping leaves the coefficient domain.
    if not df.prediction_supported.all():raise RuntimeError('A scored cruise prediction is outside the propeller table')
    df.to_csv(OUT/'predictions.csv',index=False)
    metrics=[]
    for (day,model),group in df.groupby(['date','model']):
        metrics.append(dict(date=day,partition='fit' if day==TRAIN else 'held_out',model=model,
            windows=len(group),rpm_rmse=float(np.sqrt(np.mean(group.rpm_error**2))),
            rpm_bias=float(group.rpm_error.mean()),rpm_mape_pct=float(group.relative_error_pct.abs().mean()),
            rpm_max_error_pct=float(group.relative_error_pct.abs().max()),
            climb_rmse_mps=float(np.sqrt(np.mean((group.climb_predicted-group.climb_observed)**2)))))
    metrics=pd.DataFrame(metrics);metrics.to_csv(OUT/'metrics.csv',index=False)
    return df,metrics


def lifecycle():
    f=bench(MODEL,0,2449.6953)
    rpm,converged,_=settle(f,0)
    idle=bool(converged and f['propulsion/engine/set-running'] and abs(rpm-1650)<50)
    f['propulsion/da35-ignition']=0;stopped,ok,_=settle(f,0)
    return dict(cold_start_idle_pass=idle,idle_rpm=rpm,ignition_shutdown_pass=bool(ok and stopped<1 and not f['propulsion/engine/set-running']))


def assess_run(run_dir):
    if run_dir is None:return {'status':'not_run'}
    run_dir=run_dir.resolve()
    for part in [f'{MODEL}.xml','Engine/DA35.xml','Engine/Tiguar_20x10_CCW.xml']:
        if (run_dir/'aircraft'/MODEL/part).read_bytes()!=(ROOT/'aircraft'/MODEL/part).read_bytes():raise ValueError('Run candidate mismatch')
    if (run_dir/'sitl.parm').read_text()!=default_text(MODEL):raise ValueError('Run parameters mismatch')
    summary=json.loads((run_dir/'summary.json').read_text())
    if not(summary.get('valid') and summary.get('profile') and summary.get('model')==MODEL):raise ValueError('Invalid coupled profile')
    check=subprocess.run([sys.executable,str(ROOT/'scripts/check_pipeline.py'),'--model',MODEL,'--run-dir',str(run_dir)],cwd=ROOT,text=True,capture_output=True)
    (OUT/'pipeline_checks.txt').write_text(check.stdout+check.stderr)
    telemetry=pd.read_csv(run_dir/'telemetry.csv')
    phases=[]
    for seq,alt,speed in [(3,100,23),(5,100,25),(7,130,23),(8,100,23),(9,100,23)]:
        w=telemetry[telemetry.mission_seq==seq];w=w[w.t>w.t.max()-20]
        phases.append(dict(mission_seq=seq,altitude_target_m=alt,altitude_mean_m=float(w.alt.mean()),
                           airspeed_target_mps=speed,airspeed_mean_mps=float(w.eas.mean()),throttle_mean_pct=float(w.throttle_pct.mean())))
    pd.DataFrame(phases).to_csv(OUT/'sitl_phases.csv',index=False)
    fig,axes=plt.subplots(3,1,figsize=(12,8),sharex=True)
    for ax,col,label in zip(axes,['alt','eas','throttle_pct'],['Altitude (m)','Airspeed EAS (m/s)','Throttle (%)']):
        ax.plot(telemetry.t,telemetry[col]);ax.set_ylabel(label);ax.grid(alpha=.25)
    axes[-1].set_xlabel('Simulation time (s)');fig.tight_layout();fig.savefig(OUT/'sitl_profile.png',dpi=150);plt.close(fig)
    return dict(status='passed' if check.returncode==0 else 'failed',run=str(run_dir.relative_to(ROOT)),settled_phases=phases)


def report(windows,predictions,metrics,config):
    fig,axes=plt.subplots(2,2,figsize=(13,9))
    for model,label,color in [('tig_da35','Before','#1672ad'),(MODEL,'Cruise correction','#e47b22')]:
        for date,marker in [(TRAIN,'o'),(TEST,'s')]:
            s=predictions[(predictions.model==model)&(predictions.date==date)]
            axes[0,0].scatter(s.rpm_observed,s.rpm_predicted,c=color,marker=marker,label=f'{label}: {date}')
            axes[0,1].scatter(s.throttle*100,s.rpm_error,c=color,marker=marker,label=f'{label}: {date}')
            axes[1,0].scatter(s.climb_observed,s.climb_predicted,c=color,marker=marker,label=f'{label}: {date}')
    axes[0,0].plot([4000,6500],[4000,6500],'k:');axes[0,0].set(xlabel='Observed RPM',ylabel='Steady simulated RPM')
    axes[0,1].axhline(0,color='black',ls=':');axes[0,1].set(xlabel='Logged throttle (%)',ylabel='RPM error: simulated − observed')
    axes[1,0].plot([-.5,.5],[-.5,.5],'k:');axes[1,0].set(xlabel='Observed climb (m/s)',ylabel='Energy-balance predicted climb (m/s)')
    u=np.linspace(0,1,501);axes[1,1].plot(u*100,u*100,'--',label='Before');axes[1,1].plot(u*100,effective_throttle(u,config['scale'])*100,label='Corrected input')
    axes[1,1].axvspan(config['training_throttle_range'][0]*100,config['training_throttle_range'][1]*100,alpha=.15,color='green',label='Training support')
    axes[1,1].set(xlabel='Logged throttle (%)',ylabel='Input to existing DA35 linkage (%)')
    for ax in axes.flat:ax.grid(alpha=.25);ax.legend(fontsize=7)
    fig.tight_layout();fig.savefig(OUT/'cruise_comparison.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(2,1,figsize=(12,7),sharex=True)
    ordered=windows.sort_values(['date','start_s']).reset_index(drop=True)
    for i,r in ordered.iterrows():
        axes[0].errorbar(i,r.rpm,yerr=r.rpm_std,fmt='ko',capsize=3,label='Observed mean ± within-window SD' if i==0 else None)
    for model,label in [('tig_da35','Before'),(MODEL,'Corrected')]:
        s=predictions[predictions.model==model].sort_values(['date','start_s'])
        axes[0].plot(np.arange(len(s)),s.rpm_predicted,'o',label=label)
    axes[1].plot(np.arange(len(ordered)),ordered.throttle_output_norm*100,'o-',label='Throttle')
    axes[0].set_ylabel('RPM');axes[1].set_ylabel('Throttle (%)')
    axes[1].set_xticks(np.arange(len(ordered)),[f'{r.date[5:]} F{r.flight}\n{r.start_s:.0f}s {r.kind}' for r in ordered.itertuples()],rotation=60,ha='right',fontsize=8)
    for ax in axes:ax.grid(alpha=.25);ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(OUT/'cruise_windows.png',dpi=160);plt.close(fig)
    train=windows[windows.date==TRAIN];test=windows[windows.date==TEST]
    # Main charts answer the user's question directly: actual versus simulation.
    colors={'actual':'#222b36','sim':'#e47718','before':'#2879b0'}
    chart_files=[]
    sample_tables=[]
    for day,title,filename in [(TRAIN,'May 1 · samples used to tune the model','actual_vs_simulated_may01.png'),
                               (TEST,'April 30 · samples not used to tune the correction','actual_vs_simulated_apr30.png')]:
        s=predictions[(predictions.date==day)&(predictions.model==MODEL)].sort_values('start_s').reset_index(drop=True)
        x=np.arange(1,len(s)+1)
        fig,(ax,error_ax)=plt.subplots(2,1,figsize=(12,7),sharex=True,gridspec_kw={'height_ratios':[2.2,1]},layout='constrained')
        for xi,actual,sim in zip(x,s.rpm_observed,s.rpm_predicted):
            ax.plot([xi,xi],[actual,sim],color='#b8bec6',lw=2,zorder=1)
        ax.scatter(x,s.rpm_observed,s=90,marker='o',facecolors='white',edgecolors=colors['actual'],linewidths=2,label='Actual flight: measured RPM',zorder=4)
        ax.scatter(x,s.rpm_predicted,s=65,marker='D',color=colors['sim'],label='Simulation: tuned DA35 RPM',zorder=5)
        lo=min(s.rpm_observed.min(),s.rpm_predicted.min());hi=max(s.rpm_observed.max(),s.rpm_predicted.max())
        margin=max(150,(hi-lo)*.15);ax.set_ylim(lo-margin,hi+margin)
        ax.set_ylabel('Engine speed (RPM)',fontsize=12);ax.set_title(title,loc='left',fontsize=15,pad=14)
        ax.legend(loc='best',fontsize=11);ax.grid(axis='y',alpha=.2)
        error_ax.bar(x,s.rpm_error,color=colors['sim'],width=.55)
        bound=max(30,float(s.rpm_error.abs().max())*1.35)
        error_ax.set_ylim(-bound,bound);error_ax.axhline(0,color=colors['actual'],lw=1)
        for xi,err in zip(x,s.rpm_error):
            error_ax.annotate(f'{err:+.0f}',(xi,err),xytext=(0,5 if err>=0 else -6),textcoords='offset points',ha='center',va='bottom' if err>=0 else 'top',fontsize=10)
        error_ax.set_ylabel('Simulated − actual\nRPM',fontsize=11)
        error_ax.set_xlabel('Cruise sample number · each sample is a separate 10-second average',fontsize=11)
        error_ax.set_xticks(x);error_ax.set_xlim(.4,len(s)+.6);error_ax.grid(axis='y',alpha=.2)
        fig.savefig(OUT/filename,dpi=160);plt.close(fig);chart_files.append(filename)
        samples=pd.DataFrame({'Sample':x,'Flight':s.flight,'Log time (s)':s.start_s.round(1),
                              'Actual RPM':s.rpm_observed.round(0).astype(int),'Simulated RPM':s.rpm_predicted.round(0).astype(int),
                              'Difference (RPM)':s.rpm_error.round(0).astype(int)})
        sample_tables.append(samples.to_html(index=False,border=0))
    summary=[]
    for day,label in [(TRAIN,'May 1 · fitted samples'),(TEST,'April 30 · held-out samples')]:
        r=metrics[(metrics.date==day)&(metrics.model==MODEL)].iloc[0]
        summary.append({'Data':label,'Samples':int(r.windows),'Mean absolute RPM error':f'{r.rpm_mape_pct:.1f}%',
                        'RMS RPM error':f'{r.rpm_rmse:.0f} RPM'})
    summary_table=pd.DataFrame(summary).to_html(index=False,border=0)
    takeoff_section=''
    if config['checks']['coupled'].get('run'):
        from plot_da35_takeoff import generate
        takeoff_section=generate(ROOT/config['checks']['coupled']['run'])
    html_text=f'''<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>Tiguar cruise and takeoff: actual vs simulated</title>
<style>body{{font:17px/1.6 system-ui,sans-serif;max-width:1100px;margin:36px auto;padding:0 20px;color:#202b38;background:#fafbfc}}h1,h2{{line-height:1.2}}h1{{font-size:34px}}h2{{margin-top:36px;font-size:25px}}img{{width:100%;height:auto;background:white}}figure{{margin:20px 0 32px}}figcaption{{font-size:15px;color:#4c5969}}table{{border-collapse:collapse;width:100%;background:white}}td,th{{padding:10px;border-bottom:1px solid #dce2e8;text-align:left!important}}.table-wrap{{overflow-x:auto}}.lead{{font-size:20px}}.note{{padding:16px 20px;background:#fff4df;border-left:4px solid #c47b12}}.key{{padding:14px 20px;background:#edf2f7}}.actual{{color:#222b36}}.sim{{color:#b35305}}details{{margin:20px 0;padding:16px;background:white;border:1px solid #dce2e8;border-radius:6px}}summary{{font-weight:600;cursor:pointer}}pre{{white-space:pre-wrap;font-size:14px}}code{{font-size:.9em}}</style></head><body>
<h1>Cruise and takeoff: actual vs simulation</h1>
{'<p><a href="#takeoff">Jump to takeoff: airspeed, RPM, height and controls</a></p>' if takeoff_section else ''}
{'<p><a href="pitch_investigation.html">Pitch investigation: original vs BIN-matched settings, coefficient tests and parameter audit</a></p>' if (OUT/'pitch_investigation.html').exists() else ''}
{'<p><a href="climb_options.html">Climb-rate options: tested pitch, climb-rate and throttle settings</a></p>' if (OUT/'climb_options.html').exists() else ''}
{'<p><a href="#current-default">Current default: 75% takeoff throttle and steady-cruise verification</a></p>' if (OUT/'default75_section.html').exists() else ''}
<p class="lead">The tuned DA35 now tracks the measured cruise RPM closely in these selected samples.</p>
<p class="key"><strong class="actual">○ Black circles = actual flight measurements.</strong><br><strong class="sim">◆ Orange diamonds = simulated RPM from the tuned DA35.</strong><br>The smaller the gap between the two markers, the closer the match.</p>
<div class="table-wrap">{summary_table}</div>
<p>Each point represents a separate <strong>10-second cruise average</strong>. The simulation is evaluated at that sample's recorded throttle, true airspeed and air density. These are steady engine calculations—not a replay of the flight path. Sample numbers do not represent a continuous time axis.</p>
<h2>1. Actual vs simulated: fitting samples</h2>
<p>These May 1 samples were used to adjust the throttle response, so a good fit here is expected.</p>
<figure><img src="{chart_files[0]}" alt="May 1 actual measured RPM as black circles and simulated tuned DA35 RPM as orange diamonds for 15 cruise samples, with signed RPM errors below."><figcaption>Top: compare the black circle with the orange diamond in each column. Bottom: zero means a perfect match; positive bars mean the simulation runs faster than the measurement.</figcaption></figure>
<details><summary>View the actual and simulated numbers for May 1</summary><div class="table-wrap">{sample_tables[0]}</div></details>
<h2>2. Actual vs simulated: held-out samples</h2>
<p>These April 30 samples were <strong>not used to fit the new correction</strong>. They check how it behaves on another flight day.</p>
<figure><img src="{chart_files[1]}" alt="April 30 measured and simulated RPM for two held-out cruise samples, with signed errors below."><figcaption>The markers nearly overlap. This is promising, but two samples are not enough to establish broad accuracy. The vertical scales differ from the May 1 chart; use the labeled RPM values.</figcaption></figure>
<details><summary>View the actual and simulated numbers for April 30</summary><div class="table-wrap">{sample_tables[1]}</div></details>
<p class="note"><strong>What this establishes:</strong> cruise RPM agreement improved. It does not establish accurate thrust, fuel use or takeoff performance. The held-out climb prediction still has roughly 1.5 m/s RMS error. All Tiguar profiles use the shared BIN-derived base; the cruise-tuned model now adds its approved 75% takeoff throttle limit.</p>
{takeoff_section}
{(OUT/'default75_section.html').read_text() if (OUT/'default75_section.html').exists() else ''}
<h2>What changed in the model?</h2>
<p>A single fitted factor ({config['scale']:.4f}) reduces the input to the existing DA35 throttle linkage in the cruise region. Idle and high-throttle settings are preserved. This is an empirical adjustment, not a new horsepower specification. The experimental aircraft is <code>{MODEL}</code>.</p>
<p>You recalled approximately <strong>20% electric assistance during takeoff or initial climb</strong>. That is recorded separately from the manual's nominal 50% capability. Actual shaft assistance is not verified, and no boost has been added to this gasoline-only cruise model.</p>
<details><summary>Before vs after tuning, climb estimates, and throttle mapping</summary>
<p>These are supporting diagnostics. Blue = original DA35 simulation; orange = tuned DA35 simulation. Circles identify May 1 and squares April 30. The black diagonal in the RPM panel means simulated equals actual; zero in the error panel means no RPM difference.</p>
<img src="cruise_comparison.png" alt="Technical before and after diagnostics for RPM, error, energy-balance climb, and throttle correction.">
<div class="table-wrap">{metrics.to_html(index=False,float_format=lambda x:f'{x:.2f}',border=0)}</div>
<p>{html.escape(config['interpretation'])} Climb estimates use the existing drag model with recorded acceleration and bank; they are not full-flight simulations.</p></details>
<details><summary>Sample selection and measurement variation</summary>
<p>Existing quality-filtered, nonoverlapping windows are restricted to level flight or steady turns, 21–25 m/s EAS, valid propeller advance ratio, at least 60 seconds after flight detection, and continuously normal flight stage. Climb and descent windows are excluded. No boost telemetry is available to independently verify boost-off.</p>
<p>{len(train)} fitting windows across {train.flight.nunique()} flights; {len(test)} held-out windows across {test.flight.nunique()} flight(s). Fitting throttle range: {train.throttle_output_norm.min()*100:.1f}–{train.throttle_output_norm.max()*100:.1f}%. Held-out throttle: {test.throttle_output_norm.min()*100:.1f}–{test.throttle_output_norm.max()*100:.1f}%, outside the fitting range. Actual EAS range: {windows.eas.min():.2f}–{windows.eas.max():.2f} m/s.</p>
<img src="cruise_windows.png" alt="Within-window RPM variation and recorded throttle for each selected sample.">
<p>Error bars show within-window variation, not confidence intervals. Earlier model priors and the fleet spreadsheet are not fully independent of these logs. The correction trades some fleet-average agreement for closer agreement with these particular flights.</p></details>
<details><summary>Coupled flight-test results and reproducibility</summary>
<p>The coupled run checks that the aircraft can track commanded speed and altitude. It is a separate mission, not a match to the real flight path.</p>
{'<img src="sitl_profile.png" alt="Simulated altitude, airspeed and throttle during the coupled test mission.">' if config['checks']['coupled']['status'] != 'not_run' else ''}
<pre>{html.escape(json.dumps(config['checks'],indent=2))}</pre>
<p>Redraw this report from saved results without refitting or rerunning the simulator:</p><pre>MPLCONFIGDIR=/tmp/tiguar-mpl python3 scripts/tune_da35_cruise.py --report-only</pre>
<p>Reproduce the calibration and flight check:</p><pre>MPLCONFIGDIR=/tmp/tiguar-mpl python3 scripts/tune_da35_cruise.py
python3 scripts/run_sitl.py --model tig_da35_cruise --name da35_cruise_validation --profile --duration 620
MPLCONFIGDIR=/tmp/tiguar-mpl python3 scripts/tune_da35_cruise.py --run-dir runtime/da35_cruise_validation</pre></details>
<p><a href="predictions.csv">Actual and simulated data</a> · <a href="metrics.csv">Error metrics</a> · <a href="selection_audit.csv">Selection audit</a> · <a href="cruise_windows.csv">Selected windows</a></p></body></html>'''
    (OUT/'index.html').write_text(html_text)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run-dir',type=Path)
    parser.add_argument('--report-only',action='store_true',help='Redraw saved results without refitting or running simulations')
    args=parser.parse_args()
    OUT.mkdir(parents=True,exist_ok=True)
    if args.report_only:
        saved_config=json.loads(CONFIG.read_text())
        if args.run_dir:
            saved_config['checks']['coupled']=assess_run(args.run_dir)
            saved_config['checks']['coupled']['parameters_sha256']=hashlib.sha256((args.run_dir/'sitl.parm').read_bytes()).hexdigest()
            CONFIG.write_text(json.dumps(saved_config,indent=2)+'\n')
        report(pd.read_csv(OUT/'cruise_windows.csv'),pd.read_csv(OUT/'predictions.csv'),
               pd.read_csv(OUT/'metrics.csv'),saved_config)
        print(f'Report: {OUT}/index.html')
        return
    windows=select_cruise();altitudes={}
    for rho in windows.rho.unique():
        altitudes[rho]=brentq(lambda h:bench('tig_da35',23,h)['atmosphere/rho-slugs_ft3']*RHO_CONVERSION-rho,-1000,20000)
    windows['altitude_ft']=windows.rho.map(altitudes)
    train=windows[windows.date==TRAIN]
    history=[]
    def objective(scale):
        errors=[predict('tig_da35',r,effective_throttle(r.throttle_output_norm,scale))[0]-r.rpm for r in train.itertuples()]
        mse=float(np.mean(np.square(errors)));history.append(dict(scale=scale,training_rpm_rmse=np.sqrt(mse)))
        return mse
    result=minimize_scalar(objective,bounds=(.75,1.15),method='bounded',options={'xatol':1e-5})
    if not result.success:raise RuntimeError('Fit did not converge')
    scale=float(result.x);build(scale);predictions,metrics=evaluate(windows)
    pd.DataFrame(history).to_csv(OUT/'fit_history.csv',index=False)
    train_before=float(metrics[(metrics.date==TRAIN)&(metrics.model=='tig_da35')].rpm_rmse.iloc[0]);train_after=float(metrics[(metrics.date==TRAIN)&(metrics.model==MODEL)].rpm_rmse.iloc[0])
    test_before=float(metrics[(metrics.date==TEST)&(metrics.model=='tig_da35')].rpm_rmse.iloc[0]);test_after=float(metrics[(metrics.date==TEST)&(metrics.model==MODEL)].rpm_rmse.iloc[0])
    interpretation=f'Training RPM RMSE changes from {train_before:.0f} to {train_after:.0f}; held-out RPM RMSE changes from {test_before:.0f} to {test_after:.0f}. '
    interpretation+=('The correction improves both day-level scores, but two held-out windows cannot establish broad accuracy.' if test_after<test_before else 'The correction does not improve the held-out day; do not call it a generally better model.')
    config=dict(model=MODEL,scale=scale,training=TRAIN,held_out=TEST,fit_objective='Equal-weight mean squared RPM error on May 1 selected cruise windows only',
                training_throttle_range=[float(train.throttle_output_norm.min()),float(train.throttle_output_norm.max())],
                scale_bounds=[.75,1.15],interpretation=interpretation,metrics=metrics.to_dict('records'),
                sources={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [ROOT/'aircraft/tig_da35/tig_da35.xml',ROOT/'aircraft/tig_da35/Engine/DA35.xml',ROOT/'aircraft/tig_da35/Engine/Tiguar_20x10_CCW.xml',ROOT/f'analysis/{TRAIN}/segments.csv',ROOT/f'analysis/{TEST}/segments.csv',ROOT/f'analysis/{TRAIN}/samples.csv.gz',ROOT/f'analysis/{TEST}/samples.csv.gz']},
                checks={**lifecycle(),'coupled':assess_run(args.run_dir),'default_changed':False})
    CONFIG.write_text(json.dumps(config,indent=2)+'\n');report(windows,predictions,metrics,config)
    print(interpretation);print(metrics.to_string(index=False));print(f'Report: {OUT}/index.html')

if __name__=='__main__':main()
