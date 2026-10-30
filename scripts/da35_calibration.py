#!/usr/bin/env python3
"""Build and assess a separate DA35 piston candidate; never promote the default."""
import argparse
import hashlib
import html
import json
import os
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

os.environ.setdefault('MPLCONFIGDIR', '/tmp/tiguar-mpl')
import numpy as np
import pandas as pd
from scipy.optimize import brentq
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from cruise_calibration import ROOT, read_targets, bench, settle, RHO_CONVERSION
from calibrate import D, prop_tables

MODEL = 'tig_da35'
DEST = ROOT/'aircraft'/MODEL
OUT = ROOT/'reports/da35_calibration'
CONFIG = ROOT/'config/da35_calibration.json'
ENGINE_SOURCE = ROOT/'config/DA35.xml'


def build(mapping):
    """Copy the current airframe, replacing only propulsion controls and engine."""
    source = ROOT/'aircraft/tig_calibrated'
    DEST.mkdir(parents=True, exist_ok=True)
    (DEST/'Engine').mkdir(parents=True, exist_ok=True)
    shutil.copy2(source/'Engine/Tiguar_20x10_CCW.xml', DEST/'Engine/Tiguar_20x10_CCW.xml')
    shutil.copy2(source/'reset.xml', DEST/'reset.xml')
    shutil.copy2(ENGINE_SOURCE, DEST/'Engine/DA35.xml')
    tree = ET.parse(source/'tig_calibrated.xml')
    root = tree.getroot()
    root.set('name', MODEL)
    root.find('fileheader/description').text = 'Provisional DA35 piston candidate. See config/da35_calibration.json and reports/da35_calibration/index.html. Gasoline-only approximation; electric assistance unmodeled.'
    root.find('propulsion/engine').set('file', 'DA35')
    control = root.find('flight_control')
    control.remove(control.find("channel[@name='Empirical propulsion']"))
    ET.SubElement(control, 'property', {'value':'3'}).text = 'propulsion/da35-ignition'
    ET.SubElement(control, 'property', {'value':'0.85'}).text = 'propulsion/da35-mixture'
    channel = ET.SubElement(control, 'channel', {'name':'DA35 carburetor and starting'})
    function = ET.SubElement(channel, 'fcs_function', {'name':'fcs/da35-carburetor'})
    body = ET.SubElement(function, 'function')
    table = ET.SubElement(body, 'table')
    ET.SubElement(table, 'independentVar', {'lookup':'row'}).text = 'fcs/throttle-cmd-norm'
    ET.SubElement(table, 'tableData').text = '\n'+'\n'.join(f'{x:.8f} {y:.10f}' for x,y in mapping)+'\n'
    ET.SubElement(function, 'output').text = 'fcs/throttle-pos-norm'
    for name, source_prop, target in [('mixture', 'propulsion/da35-mixture','fcs/mixture-pos-norm'),
                                      ('ignition','propulsion/da35-ignition','propulsion/magneto_cmd')]:
        fn=ET.SubElement(channel,'fcs_function',{'name':f'fcs/da35-{name}'})
        ET.SubElement(ET.SubElement(fn,'function'),'property').text=source_prop
        ET.SubElement(fn,'output').text=target
    # Simulated hand start during the first three seconds, not an automatic restart/governor.
    switch=ET.SubElement(channel,'switch',{'name':'fcs/da35-start-assist'})
    ET.SubElement(switch,'default',{'value':'0'})
    ET.SubElement(switch,'test',{'value':'1','logic':'AND'}).text='\nsimulation/sim-time-sec lt 3\npropulsion/engine/propeller-rpm lt 2000\npropulsion/da35-ignition gt 0\n'
    ET.SubElement(switch,'output').text='propulsion/starter_cmd'
    for name in ['propulsion/engine/power-hp','propulsion/engine/fuel-flow-rate-pps',
                 'propulsion/engine/set-running','propulsion/engine/map-inhg','fcs/da35-carburetor',
                 'propulsion/tank/contents-lbs']:
        ET.SubElement(root.find('output'),'property').text=name
    ET.indent(tree)
    tree.write(DEST/f'{MODEL}.xml', encoding='utf-8', xml_declaration=True)


def engine_bench(speed=23, altitude=2449.6953, initialized=True):
    f = bench(MODEL, speed, altitude)
    if initialized:
        f['propulsion/set-running'] = 0  # initialize engine zero, not shut it off
    return f


def point(f, throttle):
    rpm, converged, seconds = settle(f, throttle)
    return dict(throttle=float(throttle), rpm=float(rpm), converged=bool(converged), seconds=seconds,
                power_w=float(f['propulsion/engine/power-hp']*745.7),
                thrust_n=float(f['propulsion/engine/thrust-lbs']*4.448221615),
                running=bool(f['propulsion/engine/set-running']),
                fuel_flow_kg_s=float(f['propulsion/engine/fuel-flow-rate-pps']*.45359237),
                fuel_kg=float(f['propulsion/tank/contents-lbs']*.45359237))


def fit():
    targets=read_targets(ROOT/'rpm_cruise_by_aircraft.csv')
    targets['J']=23/(targets.rpm_mean/60*D)
    cp=prop_tables()[1]
    targets['supported']=targets.J.between(cp[0,0],cp[-1,0])
    rho=float(pd.read_csv(ROOT/'analysis/2026-05-01/segments.csv').rho.median())
    altitude=brentq(lambda h: bench('tig_calibrated',23,h)['atmosphere/rho-slugs_ft3']*RHO_CONVERSION-rho,-1000,20000)
    build([(0,0),(1,1)])
    def rpm_at(throttle, speed):
        f=engine_bench(speed, altitude)
        r=point(f,throttle)
        if not r['converged']: raise RuntimeError(f'Unsettled raw butterfly point: {r}')
        return r['rpm']
    idle=brentq(lambda t:rpm_at(t,0)-1650,0,1,xtol=1e-7)
    mapping=[(0.,idle)]
    for row in targets[targets.supported].itertuples():
        butterfly=brentq(lambda t:rpm_at(t,23)-row.rpm_mean,idle,1,xtol=1e-7)
        mapping.append((row.throttle,butterfly))
    # Preserve the measured 90% point; open the actual carburetor fully at 100%.
    # This is explicitly an unmeasured linkage extrapolation, not a 1.62 kW cap.
    mapping.append((1.,1.))
    if np.any(np.diff(np.array(mapping)[:,1])<0):raise ValueError('Nonmonotone carburetor fit')
    build(mapping)
    config=dict(model=MODEL, status='candidate_only', reference_tas_mps=23, reference_rho_kg_m3=rho,
                reference_altitude_ft=altitude, throttle_to_butterfly=mapping,
                source_sha256=hashlib.sha256((ROOT/'rpm_cruise_by_aircraft.csv').read_bytes()).hexdigest(),
                engine_sha256=hashlib.sha256(ENGINE_SOURCE.read_bytes()).hexdigest(),
                airframe_source_sha256=hashlib.sha256((ROOT/'aircraft/tig_calibrated/tig_calibrated.xml').read_bytes()).hexdigest(),
                confirmed_from_tiguar_manual_page_8={'displacement_cc':35,'cycles':2,'cylinders':1,'idle_rpm':1650,'operating_max_rpm':7800},
                assumptions=['4.5 hp retained from seed XML; rated power not verified by manufacturer.',
                  'Bore 38.1 mm assumed; stroke derived from displacement. These are not measured engine dimensions.',
                  'Manifold lag, friction, volumetric efficiency, compression and thermal parameters are unmeasured priors.',
                  'Fixed mixture 0.85; fuel consumption/temperature are model predictions, not calibrated measurements.',
                  'JSBSim piston airflow uses RPM/2 even with cycles=2; this remains a two-stroke approximation.',
                  'Nominal 23 m/s TAS and equal weights for four columns; sample counts unknown.',
                  'Zero command is static idle, not shutdown; ignition property 0 stops combustion.',
                  'Three-second starting assistance stands in for a hand start; no automatic in-flight restart.',
                  'No electric assistance in this candidate. User recalls takeoff/initial-climb boost only; exact time intervals and output unknown. Cruise calibration is conditional on excluding boosted samples.',
                  'Butterfly opens fully at 100%; 90–100% response is unmeasured.',
                  'Existing propeller sense preserved: manual CCW viewpoint is not specified.'],
                external_verification='Manufacturer product page checked; 35 cc and 1500–8200 RPM published, no horsepower rating. See config/da35_evidence.json.',
                evidence=json.loads((ROOT/'config/da35_evidence.json').read_text()))
    CONFIG.write_text(json.dumps(config,indent=2)+'\n')
    return config,targets


def diagnostics(config, targets):
    altitude=config['reference_altitude_ft']
    rows=[]
    for model in ['tig_calibrated',MODEL]:
        for speed in [0,21,23,25]:
            f=engine_bench(speed,altitude) if model==MODEL else bench(model,speed,altitude)
            for throttle in sorted(set([0,1]+targets.throttle.tolist())):
                if model==MODEL:r=point(f,throttle)
                else:
                    rpm,converged,seconds=settle(f,throttle)
                    r=dict(throttle=throttle,rpm=rpm,converged=converged,seconds=seconds,
                           power_w=f['propulsion/engine/power-hp']*745.7,
                           thrust_n=f['propulsion/engine/thrust-lbs']*4.448221615,
                           fuel_flow_kg_s=0,fuel_kg=.5,running=True)
                r.update(model=model,tas=speed,J=speed/(r['rpm']/60*D) if r['rpm']>0 else None)
                rows.append(r)
    swept=pd.DataFrame(rows)
    swept.to_csv(OUT/'propulsion_sweep.csv',index=False)
    nominal=swept[(swept.model==MODEL)&(swept.tas==23)]
    compared=targets.merge(nominal,on='throttle',validate='one_to_one')
    compared['error_pct']=100*(compared.rpm/compared.rpm_mean-1)
    compared.to_csv(OUT/'cruise_bins.csv',index=False)
    supported=compared[compared.supported]
    f=engine_bench(0,altitude,initialized=False)
    cold=point(f,0);full=point(f,1);back_idle=point(f,0)
    f['propulsion/da35-ignition']=0
    stopped=point(f,0)
    f=engine_bench(23,altitude);point(f,.45)
    f['propulsion/tank/contents-lbs']=0
    starved=point(f,.45)
    lifecycle=dict(cold_start=cold,full_static=full,return_to_idle=back_idle,
                   ignition_off=stopped,fuel_starved=starved)
    (OUT/'lifecycle.json').write_text(json.dumps(lifecycle,indent=2)+'\n')
    validation=dict(cruise_rms_error_pct=float(np.sqrt(np.mean(supported.error_pct**2))),
                    cruise_max_error_pct=float(supported.error_pct.abs().max()),
                    all_points_converged=bool(swept.converged.all()),
                    cold_start_pass=bool(cold['running'] and abs(cold['rpm']-1650)<50),
                    ignition_shutdown_pass=not stopped['running'] and stopped['rpm']<1,
                    starvation_shutdown_pass=not starved['running'] and starved['fuel_flow_kg_s']==0,
                    max_full_throttle_rpm=float(swept[(swept.model==MODEL)&(swept.throttle==1)].rpm.max()),
                    exceeds_manual_range=bool((swept[(swept.model==MODEL)&(swept.throttle==1)].rpm>7800).any()),
                    sitl_status='Not assessed by this standalone script; use --sitl-run with a completed matching coupled run.',
                    default_changed=False,
                    recommendation='Keep the current default. Review rated power, propeller loading and electric assistance; complete coupled SITL before considering promotion.')
    return swept,compared,validation


def takeoff_diagnostic(model):
    """Identical simple pilot commands for both engines, not an ArduPilot validation."""
    from cruise_calibration import jsbsim
    f=jsbsim.FGFDMExec(str(ROOT));f.set_debug_level(0)
    output=ROOT/'runtime/da35_standalone';output.mkdir(parents=True,exist_ok=True)
    f.set_output_path(str(output));f.set_aircraft_path('aircraft');f.set_engine_path(f'aircraft/{model}/Engine')
    if not f.load_model(model):raise RuntimeError('Cannot load takeoff model')
    f.disable_output();f.set_dt(.001)
    f['ic/h-sl-ft']=(273+.35)/.3048;f['ic/terrain-elevation-ft']=273/.3048
    f['ic/lat-geod-deg']=38.696;f['ic/long-gc-deg']=-94.25;f['ic/psi-true-deg']=0
    f.run_ic();rows=[]
    for step in range(60000):
        t=step*.001
        if step%50==0:
            speed=f['velocities/vc-kts']*.514444
            height=f['position/h-sl-ft']*.3048-273
            pitch_target=0 if speed<18 else 8
            throttle=float(np.clip(t-5,0,1))
            f['fcs/throttle-cmd-norm']=throttle
            f['fcs/elevator-cmd-norm']=np.clip(-.04*(pitch_target-f['attitude/theta-deg'])+.02*np.degrees(f['velocities/q-rad_sec']),-1,1)
            f['fcs/aileron-cmd-norm']=np.clip(-.04*f['attitude/phi-deg']-.015*np.degrees(f['velocities/p-rad_sec']),-1,1)
            rows.append(dict(t=t,altitude=height,eas=speed,rpm=f['propulsion/engine/propeller-rpm'],
                             throttle_output_norm=throttle,climb=-f['velocities/v-down-fps']*.3048,
                             pitch=f['attitude/theta-deg'],model=model))
            if not np.isfinite([height,speed]).all() or height < -.5 or height>500:raise RuntimeError('Invalid takeoff diagnostic: divergent or below terrain')
            if height>=50:break
        if not f.run():raise RuntimeError('Standalone JSBSim stopped')
    frame=pd.DataFrame(rows);frame.to_csv(OUT/f'{model}_takeoff.csv',index=False)
    return frame


def make_report(config,targets,swept,compared,validation):
    fig,axes=plt.subplots(2,2,figsize=(13,9))
    x=targets.throttle*100
    ax=axes[0,0];ax.fill_between(x,targets.rpm_min,targets.rpm_max,color='gray',alpha=.2,label='Fleet spread')
    ax.plot(x,targets.rpm_mean,'ko',label='CSV average')
    for model,label in [('tig_calibrated','Current equivalent engine'),(MODEL,'DA35 piston candidate')]:
        s=swept[(swept.model==model)&(swept.tas==23)]
        ax.plot(s.throttle*100,s.rpm,'o-',label=label)
    ax.axhline(7800,color='red',ls=':',label='Manual range upper value')
    ax.set(ylabel='RPM',title='Cruise at 23 m/s; full throttle unmeasured')
    ax=axes[0,1];mapping=np.array(config['throttle_to_butterfly'])
    ax.plot(mapping[:,0]*100,mapping[:,1]*100,'o-');ax.set(ylabel='Carburetor position (%)',title='Throttle linkage, not a shaft-power fraction')
    ax=axes[1,0]
    for speed in [0,21,23,25]:
        s=swept[(swept.model==MODEL)&(swept.tas==speed)]
        ax.plot(s.throttle*100,s.power_w,label=f'{speed} m/s')
    ax.set(ylabel='Shaft power (W)',title='Piston power depends on RPM and manifold pressure')
    ax=axes[1,1]
    for model,label in [('tig_calibrated','Current equivalent engine'),(MODEL,'DA35 piston candidate')]:
        s=swept[(swept.model==model)&(swept.tas==0)]
        ax.plot(s.throttle*100,s.thrust_n,'o-',label=label)
    ax.set(ylabel='Static thrust (N)',title='Predicted static thrust; not measured thrust')
    for ax in axes.flat:ax.set_xlabel('Throttle output (%)');ax.grid(alpha=.25);ax.legend(fontsize=8) if ax.get_legend_handles_labels()[0] else None
    fig.tight_layout();fig.savefig(OUT/'engine_comparison.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(4,1,figsize=(12,11),sharex=True)
    takeoff_metrics=[]
    for model in ['tig_calibrated',MODEL]:
        s=takeoff_diagnostic(model)
        selection=s[s.eas>=12]
        start=float(selection.iloc[0].t) if len(selection) else float(s.t.min())
        w=s[s.t>=start]
        for ax,col in zip(axes,['altitude','eas','rpm','throttle_output_norm']):ax.plot(w.t-start,w[col],label=f'{model}: simple pilot')
        takeoff_metrics.append(dict(model=model,seconds_from_throttle_ramp_to_50m=float(s.t.max()-5),
                                    reached_50m=bool(s.altitude.iloc[-1]>=50),max_rpm=float(s.rpm.max()),
                                    max_rpm_after_12_mps=float(w.rpm.max()),
                                    note='Identical simple pilot, flaps up; not coupled ArduPilot or real-flight validation'))
    real_notes=[]
    for date in ['2026-04-30','2026-05-01']:
        s=pd.read_csv(ROOT/f'analysis/{date}/samples.csv.gz')
        s=s[s.flight==1]
        start=float(s.t.iloc[0]);w=s[s.t<=start+30]
        w[['t','altitude','eas','rpm','throttle_output_norm','flap','stage']].to_csv(OUT/f'{date}_takeoff.csv',index=False)
        for ax,col in zip(axes,['altitude','eas','rpm','throttle_output_norm']):ax.plot(w.t-start,w[col],label=f'{date}: real, different commands')
        real_notes.append(dict(date=date,initial_flap=float(w.flap.iloc[0]),max_rpm=float(w.rpm.max())))
    for ax,label in zip(axes,['Relative altitude (m)','Airspeed EAS (m/s)','RPM','Throttle fraction']):ax.set_ylabel(label);ax.grid(alpha=.25)
    axes[0].legend(fontsize=8);axes[-1].set_xlabel('Seconds from simulation reaching 12 m/s / real flight detection')
    fig.suptitle('Takeoff context only: different controllers, flap settings, wind and possible electric assistance')
    fig.tight_layout();fig.savefig(OUT/'takeoff_context.png',dpi=150);plt.close(fig)
    validation['standalone_takeoff']=takeoff_metrics;validation['real_takeoff_context']=real_notes
    (OUT/'validation.json').write_text(json.dumps(validation,indent=2)+'\n')
    config['validation']=validation;CONFIG.write_text(json.dumps(config,indent=2)+'\n')
    (OUT/'index.html').write_text(f'''<!doctype html><html lang="en"><meta charset="utf-8"><title>Tiguar DA35 piston candidate</title>
<style>body{{font:16px/1.5 system-ui;max-width:1100px;margin:40px auto;padding:0 20px}}img{{width:100%}}table{{border-collapse:collapse;font-size:13px}}td,th{{padding:5px;border:1px solid #ccc}}pre{{white-space:pre-wrap}}</style>
<h1>Tiguar DA35 piston candidate</h1><p><strong>Candidate only. The current default has not changed.</strong> {html.escape(validation['recommendation'])}</p>
<p><a href="../da35_cruise_analysis/index.html">New: cruise-flight analysis and one-parameter empirical correction</a>. This separate variant is fitted to level/turn flight windows rather than the fleet-average spreadsheet.</p>
<h2>What changed</h2><p>The aircraft now has a separate <code>tig_da35</code> definition using a native JSBSim piston engine. A fitted throttle-to-carburetor linkage replaces the equivalent-power mapping. Power comes from JSBSim's manifold pressure, mixture, RPM and loss calculations; RPM is still determined by propeller loading. There is no forced RPM or artificial 1.62 kW cap.</p>
<p>The old DA35 file's <code>power_factor_vs_rpm</code> table is not a supported FGPiston table. Its custom <code>shaft-power-hp</code> function is not consumed by the piston power calculation. Both have been removed from the candidate. Explicit geometry, friction and starting parameters replace large-engine defaults.</p>
<h2>Evidence and assumptions</h2><p>The supplied <em>S-410 Tiguar User Manual v3.0</em>, page 8, confirms 35 cc, one cylinder, two strokes, 1650 RPM idle and a 1650–7800 RPM operating range. The <a href="https://www.desertaircraft.com/products/da-35">manufacturer product page</a> lists 1500–8200 RPM and no horsepower rating (checked October 29, 2026). The retained 4.5 hp rating is an unverified seed-file assumption. The installed variant has not been established.</p>
<p>The user recalls approximately 20% electric assistance during takeoff or initial climb only. This is an operational recollection, not a verified shaft-power measurement. Manual section 4.1 separately describes approximately 50% nominal takeoff power boost and a brushless electric motor labeled 2200 W / 230 KV. These figures must not be substituted for each other. Cruise calibration is conditional on the CSV excluding boosted climb samples. The candidate has no boost model; it must not absorb boosted takeoff output into a gasoline-only rating.</p>
<p>The bore/stroke split, losses, compression, mixture, thermal properties and manifold lag remain assumptions. JSBSim's piston model still uses a four-stroke-like airflow calculation (RPM/2) even with cycles=2, so fuel consumption and temperatures must not be treated as measured DA35 performance. Rotation sense remains unchanged because the manual does not specify its viewing direction.</p>
<h2>Cruise fit and full-throttle findings</h2><img src="engine_comparison.png">
<p>Nine supported cruise bins are fitted at 23 m/s TAS, density {config['reference_rho_kg_m3']:.4f} kg/m³. RMS relative RPM error is {validation['cruise_rms_error_pct']:.4f}%. This is a consistency check against the fitting targets, not independent validation. The three lower bins exceed the available propeller coefficient domain at cruise and are excluded.</p>
<p>Zero throttle is a running static idle; ignition off is shutdown. The 90% midpoint matches the highest CSV bin, while 100% opens the carburetor fully. That final segment is unmeasured. The candidate reaches {validation['max_full_throttle_rpm']:.0f} RPM in full-throttle bench tests, above the manual's range. JSBSim's <code>maxrpm</code> parameter is a reference for engine calculations, not an RPM governor. Do not hide this discrepancy with a fictitious limiter: rated power, installed propeller coefficients and the actual throttle linkage need review.</p>
{compared[['thr_bin','rpm_mean','supported','rpm','error_pct']].to_html(index=False,float_format=lambda x:f'{x:.3f}')}
<h2>Takeoff context</h2><img src="takeoff_context.png">
<p>The new standalone tests run both engines through identical simple pilot commands: five seconds at idle, a one-second ramp to full throttle, then an 8° pitch demand above 18 m/s. They stop at 50 m altitude. They are not ArduPilot SITL and do not establish real takeoff accuracy. Real traces begin at the existing flight-detection event; simulated traces begin at 12 m/s. Ground-roll distance and time are not comparable from these anchors.</p>
<p>Both real takeoffs use a logged flap value of 50 initially, while this model has no calibrated flap behavior. Their RPM also exceeds 7800. Electric assistance, aircraft configuration and different commands could contribute; these data cannot isolate DA35-only power. A successful simulated takeoff is therefore not a validated real takeoff match. The one-second throttle ramp is a shared test assumption, not a fitted actuator response. An instantaneous full-power step from zero RPM caused a numerical ground-contact failure in the equivalent-power baseline and was rejected as a comparison setup.</p>
<h2>Validation</h2><pre>{html.escape(json.dumps(validation,indent=2))}</pre>
<p><a href="steady_logs.html">Comparison against actual steady-flight windows</a>: separate from the CSV fit and from coupled autopilot tracking checks.</p>
<h2>Reproduce</h2><pre>MPLCONFIGDIR=/tmp/tiguar-mpl python3 scripts/da35_calibration.py
python3 scripts/test_da35.py
python3 scripts/run_sitl.py --model tig_da35 --name da35_validation --profile --duration 620</pre>
<p>Coupled-flight status: {html.escape(validation['sitl_status'])}. Successful autopilot tracking does not establish a match to the real takeoff or separate electric and gasoline shaft power.</p>
<p><a href="propulsion_sweep.csv">Sweep CSV</a> · <a href="cruise_bins.csv">Cruise-bin results</a> · <a href="lifecycle.json">Start/idle/shutdown tests</a> · <a href="validation.json">Validation details</a></p></html>''')


def assess_sitl(run_dir, validation):
    import subprocess
    run_dir=run_dir.resolve()
    for relative in [f'{MODEL}.xml','Engine/DA35.xml','Engine/Tiguar_20x10_CCW.xml']:
        if (run_dir/'aircraft'/MODEL/relative).read_bytes() != (DEST/relative).read_bytes():
            raise ValueError(f'Coupled run has different candidate file: {relative}')
    if (run_dir/'sitl.parm').read_bytes() != (ROOT/'config/sitl.parm').read_bytes():
        raise ValueError('Coupled run parameters differ')
    summary=json.loads((run_dir/'summary.json').read_text())
    if not (summary.get('valid') and summary.get('profile') and summary.get('model')==MODEL):
        raise ValueError('Run is not a completed DA35 profile')
    check=subprocess.run([sys.executable,str(ROOT/'scripts/check_pipeline.py'),'--model',MODEL,
                          '--run-dir',str(run_dir)],text=True,capture_output=True,cwd=ROOT)
    (OUT/'coupled_checks.txt').write_text(check.stdout+check.stderr)
    validation.update(sitl_status='Completed 620-second profile; '+('pipeline checks passed.' if check.returncode==0 else 'pipeline checks FAILED.'),
                      sitl_pipeline_pass=check.returncode==0,sitl_run=str(run_dir.relative_to(ROOT)))
    validation['recommendation']='Keep the current default. Review rated power, propeller loading, electric boost timing and real-flight errors before considering promotion.'
    flight=pd.read_csv(run_dir/'telemetry.csv')
    phases=[]
    for seq,alt,speed in [(3,100,23),(5,100,25),(7,130,23),(8,100,23),(9,100,23)]:
        group=flight[flight.mission_seq==seq];group=group[group.t>group.t.max()-20]
        phases.append(dict(mission_seq=seq,target_alt=alt,mean_alt=float(group.alt.mean()),
                           target_eas=speed,mean_eas=float(group.eas.mean())))
    validation['settled_phases']=phases


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fit-only',action='store_true')
    parser.add_argument('--sitl-run',type=Path,help='Assess an existing completed run after verifying candidate/parameter identity')
    args=parser.parse_args()
    OUT.mkdir(parents=True,exist_ok=True)
    config,targets=fit()
    if not args.fit_only:
        swept,compared,validation=diagnostics(config,targets)
        if args.sitl_run:assess_sitl(args.sitl_run,validation)
        make_report(config,targets,swept,compared,validation)
        print(json.dumps(validation,indent=2))
    print(f'Candidate: {DEST}; report: {OUT}/index.html')

if __name__=='__main__':main()
