#!/usr/bin/env python3
"""Generate an explicitly provisional candidate; never overwrite the seed model."""
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]

def main():
    c=json.loads((ROOT/'config/calibration.json').read_text())
    dest=ROOT/'aircraft/tig_calibrated'
    shutil.copytree(ROOT/'aircraft/tig',dest,dirs_exist_ok=True)
    tree=ET.parse(dest/'tig.xml');r=tree.getroot();r.set('name','tig_calibrated')
    r.find('fileheader/description').text='Provisional Tiguar log-conditioned candidate. See config/calibration.json and config/model_assumptions.json; CG/yaw inertia/gear geometry are priors.'
    for tag,value in [('wingarea',c['area_m2']),('wingspan',c['span_m'])]:r.find('metrics/'+tag).text=str(value)
    cr=c['root_chord_m'];ct=c['tip_chord_m'];taper=ct/cr
    r.find('metrics/chord').text=str(2*cr/3*(1+taper+taper*taper)/(1+taper))
    r.find('mass_balance/emptywt').text=str(c['mass_kg']-.55)
    # Provisional loaded CG: x=0.94 m, z=0 on the thrust/wing datum.
    # Compensate the seed's 0.05 kg payload and 0.5 kg fuel point masses.
    empty=c['mass_kg']-.55
    r.find("mass_balance/location[@name='CG']/x").text=str((.94*c['mass_kg']-.05*.10-.5*.55)/empty)
    r.find("mass_balance/location[@name='CG']/z").text=str(-.5*.05/empty)
    # Loaded diagonal inertia: use supplied gear-in pitch and roll figures;
    # provisional yaw=34 satisfies the inertia triangle and allows finite vertical mass spread.
    inertias=[20.5,15.5,34.0]
    positions=[(empty,float(r.find("mass_balance/location[@name='CG']/x").text),float(r.find("mass_balance/location[@name='CG']/z").text)),(.05,.10,0),(.5,.55,.05)]
    corrections=[sum(m*z*z for m,x,z in positions),sum(m*((x-.94)**2+z*z) for m,x,z in positions),sum(m*(x-.94)**2 for m,x,z in positions)]
    for tag,target,correction in zip(['ixx','iyy','izz'],inertias,corrections):r.find('mass_balance/'+tag).text=str(target-correction)
    # Replace the seed taildragger layout with a provisional tricycle layout.
    for contact,x,y in zip(r.find('ground_reactions'),[1.10,1.10,.35],[-.3565,.3565,0]):
        loc=contact.find('location');loc.set('unit','M')
        for tag,value in [('x',x),('y',y),('z',-.35)]:loc.find(tag).text=str(value)
        contact.find('rolling_friction').text='.03'
        if y==0:contact.set('name','NOSE_LG');contact.find('max_steer').text='25'
    # Fix cancellation of opposed aileron deflections in the unused effective-aileron output.
    r.find("flight_control/channel/summer[@name='Effective Aileron Pos']").findall('input')[1].text='-fcs/right-aileron-pos-rad'
    # Elevator drag must be even in deflection, never a negative drag contribution.
    drag=r.find("aerodynamics/axis[@name='DRAG']/function[@name='aero/coefficient/CDde']/product")
    p=drag.find("property[.='fcs/elevator-pos-norm']");drag.remove(p)
    square=ET.SubElement(drag,'pow');ET.SubElement(square,'property').text='fcs/elevator-pos-norm';ET.SubElement(square,'value').text='2'
    cd=r.find("aerodynamics/axis[@name='DRAG']/function[@name='aero/coefficient/CD0']/product")
    cd.remove(cd.find('table'));ET.SubElement(cd,'value').text=str(c['cd0'])
    # Larger provisional inertias require corresponding effective moment derivatives.
    # Preserve the stable seed's approximate angular response rather than retune ArduPilot.
    # These scales are simulation priors, not independently identified aerodynamic derivatives.
    moment_scales={'ROLL':20.5/4.643,'PITCH':15.5/4.101,'YAW':34/4.588}
    for axis,scale in moment_scales.items():
        for func in r.findall(f"aerodynamics/axis[@name='{axis}']/function"):
            ET.SubElement(func.find('product'),'value').text=str(scale)
    # Keep existing equivalent control effectiveness/travel pending actuator response evidence.
    # Absorb the measured throttle-to-power nonlinearity into an equivalent shaft-power source.
    channel=ET.SubElement(r.find('flight_control'),'channel',{'name':'Empirical propulsion'})
    func=ET.SubElement(channel,'fcs_function',{'name':'fcs/effective-power-fraction'})
    function=ET.SubElement(func,'function');table=ET.SubElement(function,'table')
    ET.SubElement(table,'independentVar',{'lookup':'row'}).text='fcs/throttle-cmd-norm'
    maxpower=c['shaft_power_w'][-1]
    ET.SubElement(table,'tableData').text='\n'+'\n'.join(f'{t:.6f} {p/maxpower:.8f}' for t,p in zip(c['throttle_grid'],c['shaft_power_w']))+'\n'
    ET.SubElement(func,'output').text='fcs/throttle-pos-norm'
    engine=ET.Element('electric_engine',{'name':'Tiguar_equivalent_shaft_power'})
    ET.SubElement(engine,'power',{'unit':'WATTS'}).text=str(maxpower)
    ET.indent(engine);ET.ElementTree(engine).write(dest/'Engine/empirical_power.xml',encoding='utf-8',xml_declaration=True)
    r.find('propulsion/engine').set('file','empirical_power')
    prop=ET.parse(dest/'Engine/Tiguar_20x10_CCW.xml');pr=prop.getroot()
    # Existing comment intended kg*m2, but missing unit was interpreted as slug*ft2.
    pr.find('ixx').set('unit','KG*M2')
    ET.SubElement(pr,'ct_factor').text=str(c['thrust_scale'])
    ET.indent(prop);prop.write(dest/'Engine/Tiguar_20x10_CCW.xml',encoding='utf-8',xml_declaration=True)
    output=ET.SubElement(r,'output',{'name':'jsbsim-state.csv','type':'CSV','rate':'20'})
    for name in ['attitude/theta-deg','attitude/phi-deg','aero/alpha-deg','velocities/vtrue-fps',
                 'fcs/elevator-cmd-norm','fcs/elevator-pos-rad','fcs/throttle-cmd-norm','fcs/throttle-pos-norm',
                 'propulsion/engine/propeller-rpm','propulsion/engine/thrust-lbs',
                 'moments/m-aero-lbsft','moments/m-prop-lbsft','moments/m-total-lbsft',
                 'aero/coefficient/Cm0','aero/coefficient/Cmalpha','aero/coefficient/Cmde','aero/coefficient/Cmq']:
        ET.SubElement(output,'property').text=name
    ET.indent(tree);tree.write(dest/'tig_calibrated.xml',encoding='utf-8',xml_declaration=True)
    (dest/'tig.xml').unlink()
    assumptions={'loaded_mass_kg':c['mass_kg'],'loaded_cg_m':[.94,0,0],
        'loaded_diagonal_inertia_kgm2':inertias,
        'moment_coefficient_scales':moment_scales,
        'moment_scale_note':'Effective moment functions scaled with inertia to preserve the seed angular response; these are priors, not independently flight-identified coefficients. Logged controller gains are unchanged.',
        'inertia_note':'Roll 20.5 and pitch 15.5 from supplied document; yaw 34.0 is a physically admissible prior, NOT the measured 41.4. Rig/configuration unknown. Cross inertias remain approximately zero.',
        'cg_note':'Unmeasured: nominal x datum retained, vertical CG aligned with thrust datum. Initial seed offset caused pitch-up/stall.',
        'gear_note':'Gear included in 55 lb. Tricycle layout from manual; 0.713 m track from drawing. Fore-aft contact positions and 0.35 m height are provisional. Spring/damping retained; rolling friction reduced to 0.03.',
        'controls_note':'Equivalent elevator/rudder used; ±20 degree seed travel retained, not physical ±28 degree ruddervator travel. Flaps excluded from calibration; no validated landing model.'}
    (ROOT/'config/model_assumptions.json').write_text(json.dumps(assumptions,indent=2))
    # Extract flight-controller settings only; hardware output assignments are intentionally remapped.
    p=json.loads((ROOT/'analysis/2026-05-01/parameters_final.json').read_text())
    prefixes=('RLL_','RLL2SRV_','PTCH_','PTCH2SRV_','TECS_','NAVL1_','AIRSPEED_')
    selected={k:v for k,v in p.items() if k.startswith(prefixes)}
    selected.update({'SERVO1_FUNCTION':4,'SERVO2_FUNCTION':19,'SERVO3_FUNCTION':70,'SERVO4_FUNCTION':21,
                     'SERVO1_REVERSED':0,'SERVO2_REVERSED':1,'SERVO3_REVERSED':0,'SERVO4_REVERSED':1,
                     'ARSPD_USE':1,'TRIM_THROTTLE':p['TRIM_THROTTLE'],'THR_MAX':100,'THR_MIN':0,
                     'ROLL_LIMIT_DEG':35,'WP_LOITER_RAD':170,'LOG_DISARMED':1,'LOG_BITMASK':65535,
                     'SIM_WIND_SPD':0,'SIM_WIND_TURB':0,'TERRAIN_ENABLE':0,
                     'ARMING_CHECK':0,'FS_GCS_ENABL':0,'THR_FAILSAFE':0,'FLTMODE_CH':0,
                     'TKOFF_THR_MINACC':0,'TKOFF_THR_MINSPD':0})
    for ch in range(1,5):
        selected.update({f'SERVO{ch}_MIN':1000,f'SERVO{ch}_MAX':2000,f'SERVO{ch}_TRIM':1000 if ch==3 else 1500})
    (ROOT/'config/sitl.parm').write_text('# SIMULATION ONLY: equivalent axes on outputs 1-4; do not load onto hardware.\n'+''.join(f'{k} {v:g}\n' for k,v in sorted(selected.items())))
    print('Wrote',dest)

if __name__=='__main__':main()
