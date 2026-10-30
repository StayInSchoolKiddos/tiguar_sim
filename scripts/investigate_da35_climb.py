#!/usr/bin/env python3
"""Separate climb-control experiments; preserve the shared BIN-matched configuration."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
os.environ.setdefault('MPLCONFIGDIR','/tmp/tiguar-mpl')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pandas as pd
from report_da35_pitch import load
from plot_da35_takeoff import prepare
ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'runtime/da35_bin_matched_final'
WORK=ROOT/'runtime/da35_climb_options'
OUT=ROOT/'reports/da35_cruise_analysis'
CASES={
    'climb_limit_2':({'TECS_CLMB_MAX':2},12),
    'mission_pitch_5':({},5),
    'pitch_cap_10':({'TECS_PITCH_MAX':10},5),
    'throttle_cap_75':({'TKOFF_THR_MAX':75},12),
}
LABELS={'baseline':'BIN-matched baseline','climb_limit_2':'TECS climb limit 2 m/s',
        'mission_pitch_5':'Mission minimum pitch 5°','pitch_cap_10':'Pitch max 10°, mission min 5°',
        'throttle_cap_75':'Takeoff throttle maximum 75%'}

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',action='store_true');a=p.parse_args()
    WORK.mkdir(parents=True,exist_ok=True)
    for part in ['tig_da35_cruise.xml','Engine/DA35.xml','Engine/Tiguar_20x10_CCW.xml']:
        assert (BASE/'aircraft/tig_da35_cruise'/part).read_bytes()==(ROOT/'aircraft/tig_da35_cruise'/part).read_bytes()
    manifest={}
    for case,(settings,pitch) in CASES.items():
        setup=WORK/'inputs'/case;setup.mkdir(parents=True,exist_ok=True)
        lines=[line for line in (BASE/'sitl.parm').read_text().splitlines() if not line.split() or line.split()[0] not in settings]
        params=setup/'sitl.parm';params.write_text('\n'.join(lines)+'\n'+''.join(f'{k} {v}\n' for k,v in settings.items()))
        manifest[case]={'overrides':settings,'mission_min_pitch_deg':pitch,'parameter_sha256':hashlib.sha256(params.read_bytes()).hexdigest()}
        if a.run:
            print('Running climb option:',case,flush=True)
            with (setup/'run_console.txt').open('w') as log:
                subprocess.run([sys.executable,str(ROOT/'scripts/run_sitl.py'),'--model','tig_da35_cruise',
                    '--name',f'da35_climb_options/{case}','--param-file',str(params),'--takeoff-pitch',str(pitch),
                    '--duration','75','--profile'],check=True,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
    for case,run in [('baseline',BASE),*[(c,WORK/c) for c in CASES]]:
        manifest.setdefault(case,{})
        manifest[case].update({'run':str(run.relative_to(ROOT)),
            'bin_sha256':hashlib.sha256((run/'logs/00000001.BIN').read_bytes()).hexdigest(),
            'model_sha256':hashlib.sha256((run/'aircraft/tig_da35_cruise/tig_da35_cruise.xml').read_bytes()).hexdigest(),
            'parameter_sha256':hashlib.sha256((run/'sitl.parm').read_bytes()).hexdigest()})
    manifest['actual']={'source':'analysis/2026-05-01/samples.csv.gz','sha256':hashlib.sha256((ROOT/'analysis/2026-05-01/samples.csv.gz').read_bytes()).hexdigest()}
    (OUT/'climb_options_provenance.json').write_text(json.dumps(manifest,indent=2)+'\n')
    data={'baseline':load(BASE)[0]}
    data.update({case:load(WORK/case)[0] for case in CASES})
    actual,_=prepare(pd.read_csv(ROOT/'analysis/2026-05-01/samples.csv.gz'))
    rows=[]
    for case,s in [('actual',actual),*data.items()]:
        w=s[s.elapsed_s.between(0,60)];takeoff=w[(w.stage==1)&w.height_m.between(10,80)]
        reached=w[w.height_m>=50]
        rows.append({'case':case,'peak_climb_0_30_mps':w[w.elapsed_s<=30].climb.max(),
            'mean_takeoff_climb_at_10_80m_mps':takeoff.climb.mean(),'peak_pitch_0_30_deg':w[w.elapsed_s<=30].pitch.max(),
            'max_eas_0_60_mps':w.eas.max(),'min_eas_above_10m_mps':w[w.height_m>10].eas.min(),
            'seconds_to_50m':None if reached.empty else reached.elapsed_s.iloc[0]})
        s.to_csv(OUT/f'climb_option_{case}.csv',index=False)
    metrics=pd.DataFrame(rows);metrics.to_csv(OUT/'climb_options.csv',index=False)
    for col,label,filename in [('climb','Climb rate (m/s)','climb_options.png'),
                                ('eas','Airspeed EAS (m/s)','climb_options_airspeed.png'),
                                ('height_m','Relative height (m)','climb_options_height.png')]:
        fig,axes=plt.subplots(2,2,figsize=(13,8),sharex=True,sharey=True,layout='constrained')
        for ax,case in zip(axes.flat,CASES):
            for frame,name,color,style in [(actual,'Actual May 1','#222b36','-'),
                       (data['baseline'],'BIN-matched baseline','#e47718','--'),
                       (data[case],'This option','#087e8b','-.')]:
                ax.plot(frame.elapsed_s,frame[col],label=name,color=color,ls=style,lw=1.8)
            ax.set_title(LABELS[case],fontsize=12);ax.set_xlim(0,40);ax.grid(alpha=.2)
            ax.set_ylabel(label);ax.set_xlabel('Seconds from throttle onset')
        axes[0,0].legend(fontsize=9)
        fig.suptitle('Climb-control experiments: compare teal with orange and black')
        fig.savefig(OUT/filename,dpi=150);plt.close(fig)
    view=metrics.copy()
    view['case']=view['case'].map({'actual':'Actual May 1',**LABELS})
    view=view.rename(columns={'case':'Option','peak_climb_0_30_mps':'Peak climb, 0–30 s (m/s)',
        'mean_takeoff_climb_at_10_80m_mps':'Mean takeoff climb at 10–80 m (m/s)',
        'peak_pitch_0_30_deg':'Peak pitch, 0–30 s (deg)','max_eas_0_60_mps':'Max EAS, 0–60 s (m/s)',
        'min_eas_above_10m_mps':'Min EAS above 10 m (m/s)','seconds_to_50m':'Seconds to +50 m'})
    text=f'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>Tiguar climb-rate options</title>
<style>body{{font:17px/1.6 system-ui;max-width:1100px;margin:32px auto;padding:0 20px;color:#202b38}}img{{width:100%}}table{{border-collapse:collapse}}td,th{{padding:8px;border-bottom:1px solid #ccc;text-align:left!important}}.scroll{{overflow-x:auto}}.note{{background:#fff4df;padding:16px}}details{{margin:20px 0}}pre{{white-space:pre-wrap}}</style>
<p><a href="index.html">Cruise and takeoff report</a> · <a href="pitch_investigation.html">Pitch and parameter investigation</a></p>
<h1>Options to reduce simulated takeoff climb</h1>
<p><strong>The tested 75% takeoff throttle limit is now the default for <code>tig_da35_cruise</code>.</strong> See <a href="default75.html">the full-mission and straight-cruise verification</a>. The baseline in these experiments is the earlier BIN-matched configuration before that model override. The actual flight used different commands and configuration, so matching climb with less commanded throttle would not validate the engine model.</p>
<div class="scroll">{view.to_html(index=False,float_format=lambda x:f'{x:.2f}',border=0)}</div>
<p>Peak climb and pitch use the first 30 seconds from sustained throttle onset. Mean climb uses only takeoff-stage samples at 10–80 m above each pre-takeoff reference. These windows differ in time and duration between flights. Airspeed extrema use the displayed first 60 seconds, with the minimum restricted to height above 10 m. The actual data are the first May 1 takeoff; each simulation is a separate mission, not a replay.</p>
<img src="climb_options.png" alt="Four separate climb-rate comparisons: actual in black, BIN-matched baseline in orange, and each experimental option in teal.">
<h2>Which option addresses which problem?</h2>
<ol><li><strong>Limit takeoff throttle:</strong> set <code>TKOFF_THR_MAX=75</code> in a separate simulation profile. It caps the commanded throttle during takeoff; 75% command is not 75% shaft power. This tested setting approaches the recorded climb rate, but changes the inputs from the actual flight and increases time to +50 m. Cruise settings are unchanged.</li>
<li><strong>Reduce commanded pitch:</strong> the AUTO mission's takeoff minimum is 12°. Lowering that minimum to 5° alone does not restrict a higher TECS demand. Adding <code>TECS_PITCH_MAX=10</code> limits the navigation pitch request, with the existing +4° pitch trim still applied downstream. Pitch improves, but excess power produces more airspeed: almost 30 m/s in this test. This is not an effective standalone way to match climb.</li>
<li><strong>Lower the TECS climb setting:</strong> <code>TECS_CLMB_MAX=2</code> does not impose a 2 m/s takeoff cap in this pinned ArduPlane 4.5.6 run. Takeoff pitch prioritizes airspeed, and the takeoff logic imposes a minimum pitch. Its role in ordinary altitude tracking differs from takeoff. Changing <code>TKOFF_LVL_PITCH</code> would target TAKEOFF mode rather than the AUTO mission command used here.</li>
<li><strong>Improve physical fidelity while keeping actual parameters:</strong> retain the shared BIN-matched configuration, match the real mission/controls and loading, then investigate full-power shaft output, propeller thrust, drag and flap effects. The model's 4.5 hp rating is an unverified seed assumption. Cruise RPM calibration does not identify full-throttle thrust. Do not use a climb match alone to select new aerodynamic coefficients.</li></ol>
<p>ArduPilot's <a href="https://ardupilot.org/plane/docs/automatic-takeoff.html">automatic takeoff documentation</a> describes the takeoff pitch and throttle controls. The tested behavior above is checked against the pinned 4.5.6 source: <a href="../../vendor/ardupilot/ArduPlane/takeoff.cpp">takeoff pitch selection</a> and <a href="../../vendor/ardupilot/libraries/AP_TECS/AP_TECS.cpp">TECS takeoff speed weighting and limits</a>. Newer documentation includes controls unavailable in this pinned version.</p>
<h2>Airspeed tradeoff</h2><img src="climb_options_airspeed.png" alt="Airspeed comparison for each option; limiting pitch increases airspeed substantially.">
<details><summary>Height gained over time</summary><img src="climb_options_height.png" alt="Relative height over time for each climb-control option."></details>
<p class="note">The shared settings in <code>config/sitl.parm</code> remain matched to the BIN snapshot. <code>config/models/tig_da35_cruise.parm</code> adds the approved 75% takeoff limit for this model. No engine or aerodynamic coefficients were changed. The experiments do not establish broad fidelity or validate landing behavior, flap aerodynamics or electric assistance.</p>
<p>To repeat the tested throttle-limited simulation with its isolated parameter file:</p>
<pre>python3 scripts/run_sitl.py --model tig_da35_cruise --name my_climb_test --param-file runtime/da35_climb_options/inputs/throttle_cap_75/sitl.parm --profile --duration 75</pre>
<p>Recreate all four experiments: <code>python3 scripts/investigate_da35_climb.py --run</code>. Redraw saved results without flying again: <code>python3 scripts/investigate_da35_climb.py</code>.</p>
<p><a href="climb_options.csv">Metrics</a> · <a href="climb_options_provenance.json">Experimental settings</a> · <a href="climb_option_actual.csv">Actual samples</a> · <a href="climb_option_baseline.csv">Baseline samples</a> · <a href="climb_option_throttle_cap_75.csv">Throttle-limited samples</a></p></html>'''
    (OUT/'climb_options.html').write_text(text)
    print(metrics.round(2).to_string(index=False))

if __name__=='__main__':main()
