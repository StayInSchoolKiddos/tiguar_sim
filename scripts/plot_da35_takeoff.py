#!/usr/bin/env python3
"""Event-aligned actual versus coupled-simulation takeoff plots; not command replay."""
import argparse
import hashlib
import html
import json
import os
from pathlib import Path
import numpy as np
import pandas as pd
os.environ.setdefault('MPLCONFIGDIR','/tmp/tiguar-mpl')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'reports/da35_cruise_analysis'
DURATION=60.
PRE=5.
COLORS={'actual':'#222b36','simulated':'#e47718'}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def onset(data):
    """First sustained >=20% output near the first detected takeoff, with no outcome fit."""
    airborne=data[data.flight==1]
    if airborne.empty:raise ValueError('No first flight')
    detected=float(airborne.t.min())
    nearby=data[data.t.between(detected-30,detected+2)].reset_index(drop=True)
    for i,row in nearby.iterrows():
        if not (row.throttle_output_norm>=.2 and row.stage==1 and row.armed==1 and row.eas<8):continue
        following=nearby.iloc[i:i+11]
        previous=nearby.iloc[max(0,i-10):i]
        if (len(following)==11 and .99<=following.t.iloc[-1]-row.t<=1.01
                and (following.throttle_output_norm>=.2).all()
                and len(previous)>0 and (previous.throttle_output_norm<.2).any()):
            return float(row.t),detected
    raise ValueError('Cannot identify sustained takeoff throttle onset')


def prepare(data):
    anchor,detected=onset(data)
    ref=data[data.t.between(anchor-5,anchor-1)].altitude.dropna()
    if len(ref)<20:raise ValueError('Insufficient pre-takeoff altitude reference')
    w=data[data.t.between(anchor-PRE-1e-5,anchor+DURATION+1e-5)].copy()
    w['elapsed_s']=w.t-anchor
    w['height_m']=w.altitude-float(ref.median())
    w['throttle_output_pct']=w.throttle_output_norm*100
    return w,{'throttle_onset_s':anchor,'flight_detection_s':detected,
              'altitude_reference_m':float(ref.median()),'duration_s':DURATION}


def load_simulation(run):
    summary=json.loads((run/'summary.json').read_text())
    if summary.get('model')!='tig_da35_cruise' or not summary.get('valid'):
        raise ValueError('Expected a valid coupled tig_da35_cruise run')
    for name in ['tig_da35_cruise.xml','Engine/DA35.xml','Engine/Tiguar_20x10_CCW.xml']:
        if digest(run/'aircraft/tig_da35_cruise'/name)!=digest(ROOT/'aircraft/tig_da35_cruise'/name):
            raise ValueError(f'Saved flight uses a different model: {name}')
    binary=run/'logs/00000001.BIN'
    cache=ROOT/'analysis/da35_cruise_takeoff'
    provenance=cache/'takeoff_source.json'
    bin_hash=digest(binary)
    if not provenance.exists() or json.loads(provenance.read_text()).get('sha256')!=bin_hash:
        from extract_logs import extract
        extract(binary,cache)
        provenance.write_text(json.dumps({'source':str(binary.relative_to(ROOT)),'sha256':bin_hash},indent=2)+'\n')
    data=pd.read_csv(cache/'samples.csv.gz')
    truth=pd.read_csv(run/'jsbsim-state.csv')
    key='/fdm/jsbsim/propulsion/engine/propeller-rpm'
    truth=truth[['Time',key]].rename(columns={'Time':'truth_time_s',key:'rpm_truth'})
    # ArduPilot's JSBSim connector uses fdm.cur_time as simulation time. Use the
    # shared clock directly, with nearest truth sample <=30 ms; no fitted offset.
    data=pd.merge_asof(data.sort_values('t'),truth.sort_values('truth_time_s').drop_duplicates('truth_time_s'),
                       left_on='t',right_on='truth_time_s',direction='nearest',tolerance=.03)
    data['rpm']=data.rpm_truth
    return data,{'run':str(run.relative_to(ROOT)),'log_sha256':bin_hash,
                 'parameters_sha256':digest(run/'sitl.parm'),
                 'truth_sha256':digest(run/'jsbsim-state.csv'),
                 'rpm_source':'JSBSim propeller-rpm on shared simulation clock; nearest match within 0.03 s'}


def panel(real,sim,date,signals,filename,title):
    fig,axes=plt.subplots(len(signals),1,figsize=(12,2.6*len(signals)+.8),sharex=True,layout='constrained')
    for ax,(column,label) in zip(axes,signals):
        for name,data,style in [('actual',real,'-'),('simulated',sim,'--')]:
            ax.plot(data.elapsed_s,data[column],style,color=COLORS[name],lw=1.8,
                    label='Actual flight' if name=='actual' else 'Simulation: tuned DA35, no boost')
        ax.axvline(0,color='#718096',ls=':',lw=1)
        ax.axvspan(-PRE,0,color='#edf2f7',zorder=0)
        ax.set_ylabel(label,fontsize=12);ax.grid(alpha=.2);ax.set_xlim(-PRE,DURATION)
        if column=='throttle_output_pct':ax.set_ylim(-3,105)
    axes[0].legend(loc='best',fontsize=10)
    axes[-1].set_xlabel('Seconds from sustained takeoff throttle application · 0 = onset in each flight',fontsize=11)
    fig.suptitle(f'{title}\n{date}, first recorded takeoff vs coupled ArduPilot/JSBSim flight',fontsize=15)
    fig.savefig(OUT/filename,dpi=160);plt.close(fig)


def statistics(w,label,date):
    w=w[w.elapsed_s>=0]
    def crossing(col,value):
        found=w[w[col]>=value]
        return None if found.empty else float(found.elapsed_s.iloc[0])
    return dict(date=date,source=label,peak_rpm=float(w.rpm.max()),
                seconds_to_20mps=crossing('eas',20),seconds_to_50m=crossing('height_m',50),
                initial_flap_command=float(w.flap.iloc[0]),rpm_missing_samples=int(w.rpm.isna().sum()))


def generate(run=None):
    OUT.mkdir(parents=True,exist_ok=True)
    if run is None:
        config=json.loads((ROOT/'config/da35_cruise_tuning.json').read_text())
        run=ROOT/config['checks']['coupled']['run']
    run=Path(run).resolve()
    sim_all,provenance=load_simulation(run)
    sim,sim_anchor=prepare(sim_all)
    if sim.elapsed_s.min()>-4.9 or sim.elapsed_s.max()<59.9:raise ValueError('Incomplete simulated takeoff interval')
    if sim.rpm.isna().any():raise ValueError('Missing JSBSim RPM in displayed takeoff interval')
    if ((sim.t-sim.truth_time_s).abs()>.03).any():raise ValueError('RPM timestamp alignment failed')
    metadata={'simulation':provenance,'simulation_alignment':sim_anchor,
              'rule':'First >=20% throttle for 1 second in takeoff stage with airspeed <8 m/s, within 30 seconds before first flight detection. No fitted time shift or time warping.',
              'height_reference':'Median EKF altitude from -5 to -1 seconds before each throttle onset.',
              'comparison':'Different missions, control demands, flap settings and propulsion; descriptive overlays, not controlled fidelity scoring.',
              'actual':{}}
    metrics=[];sections=[]
    columns=['elapsed_s','t','eas','rpm','height_m','climb','pitch','roll','groundspeed','throttle_output_pct','flap','stage']
    sim[columns+['truth_time_s']].to_csv(OUT/'takeoff_simulated.csv',index=False)
    for date in ['2026-05-01','2026-04-30']:
        source=ROOT/f'analysis/{date}/samples.csv.gz'
        real,anchor=prepare(pd.read_csv(source))
        if real.elapsed_s.min()>-4.9 or real.elapsed_s.max()<59.9:raise ValueError('Incomplete real takeoff interval')
        metadata['actual'][date]={**anchor,'source_sha256':digest(source)}
        real[columns].to_csv(OUT/f'takeoff_actual_{date}.csv',index=False)
        metrics.extend([statistics(real,'Actual',date),statistics(sim,'Simulated',date)])
        primary=f'takeoff_{date}_actual_vs_simulated.png'
        controls=f'takeoff_{date}_controls.png'
        panel(real,sim,date,[('eas','Airspeed EAS (m/s)'),('rpm','Engine / propeller RPM'),('height_m','Height above pre-takeoff\nreference (m)')],primary,'Takeoff: actual vs simulated')
        panel(real,sim,date,[('climb','Climb rate (m/s)'),('pitch','Pitch attitude (degrees)'),('throttle_output_pct','Gasoline throttle output (%)')],controls,'Takeoff: climb, attitude and throttle')
        datestring='May 1' if date.endswith('05-01') else 'April 30'
        sections.append(f'''<h3>{datestring}: first recorded takeoff</h3>
<figure><img src="{primary}" alt="{datestring} actual versus simulated takeoff airspeed, RPM and height over time."><figcaption>Black solid = actual flight. Orange dashed = the same coupled simulated takeoff used for both date comparisons. Gray shading shows five seconds before throttle onset. Height is zeroed to each flight's pre-takeoff reference.</figcaption></figure>
<details open><summary>{datestring}: climb rate, pitch and throttle</summary><img src="{controls}" alt="{datestring} actual versus simulated climb rate, pitch and gasoline throttle commands."><p>The throttle traces show why these are not identical-input tests. Recorded throttle represents the gasoline output channel; it does not measure electric assistance. The simulation controller can command a different throttle or pitch while flying its own mission.</p></details>''')
    (OUT/'takeoff_alignment.json').write_text(json.dumps(metadata,indent=2)+'\n')
    pd.DataFrame(metrics).to_csv(OUT/'takeoff_summary.csv',index=False)
    frame=pd.DataFrame(metrics).rename(columns={'date':'Flight day','source':'Source','peak_rpm':'Peak RPM (0–60 s)',
            'seconds_to_20mps':'Seconds to 20 m/s EAS','seconds_to_50m':'Seconds to +50 m',
            'initial_flap_command':'Initial flap command (logged units)','rpm_missing_samples':'Missing RPM samples'})
    fragment=f'''<section id="takeoff"><h2>3. Takeoff: actual flight vs simulated flight</h2>
<p>These plots use the <strong>saved coupled ArduPilot/JSBSim run of the cruise-tuned DA35</strong>: <code>{html.escape(str(run.relative_to(ROOT)))}</code>. Each day shows its first recorded takeoff against the same simulated takeoff. The parameter-file hash is included in the provenance.</p>
<p class="key"><strong>Black solid line = actual flight.</strong><br><strong class="sim">Orange dashed line = simulated flight, without electric boost.</strong></p>
<p>Time zero is independently detected when throttle reaches 20% and stays above it for one second during takeoff. We show five seconds before and 60 seconds after this event. This is throttle onset—not liftoff. Airspeed is EAS for both traces; simulated RPM comes from JSBSim truth on the shared simulation clock.</p>
<p class="note"><strong>Different takeoff conditions:</strong> you recall approximately 20% electric assistance during takeoff/initial climb. The simulation has no boost. Actual logs show an initial flap command of 50; the simulated aircraft has no calibrated flap model. Wind, commands, targets and aircraft configuration can also differ. These overlays reveal differences; they do not isolate engine error or prove takeoff fidelity.</p>
<p><strong>What to look for:</strong> compare the initial RPM rise and acceleration, then the rate of height gain. The pitch and throttle panels help explain when different controller commands contribute to separation. Once the simulation reaches its altitude target, it reduces throttle; later RPM differences also reflect that different flight path.</p>
{''.join(sections)}
<details><summary>Takeoff timing, peak RPM and source data</summary><p>Descriptive values over the displayed interval. A blank timing means the threshold was not reached within 60 seconds. Values include recorded transients; no curve shifts or smoothing were fitted to improve agreement.</p><div class="table-wrap">{frame.to_html(index=False,float_format=lambda x:f'{x:.1f}',na_rep='Not reached',border=0)}</div>
<p><a href="takeoff_simulated.csv">Simulated data</a> · <a href="takeoff_actual_2026-05-01.csv">May 1 actual data</a> · <a href="takeoff_actual_2026-04-30.csv">April 30 actual data</a> · <a href="takeoff_alignment.json">Alignment and provenance</a></p>
<p>Simulation: <code>{html.escape(str(run.relative_to(ROOT)))}</code>. Flight-controller states are extracted from the simulated BIN log using the same fields as the actual logs. RPM is joined from JSBSim truth within 30 ms, with no clock offset fitted.</p></details></section>'''
    (OUT/'takeoff_section.html').write_text(fragment)
    return fragment


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run-dir',type=Path);args=parser.parse_args()
    generate(args.run_dir)
    print(f'Wrote takeoff plots and data to {OUT}')

if __name__=='__main__':main()
