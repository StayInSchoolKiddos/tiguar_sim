#!/usr/bin/env python3
"""JSBSim equilibrium sweeps (not transient validation).

JSBSim's built-in engine trim marches at 0.5 s, unstable for this tiny propeller
inertia. Temporary trim-only copies use 0.05 kg*m2 rotor inertia. This changes
convergence, not equilibrium power/thrust. Flight runs retain actual model files.
"""
import json
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'.deps'))
import jsbsim
import numpy as np
import pandas as pd

def setup(model):
    base=ROOT/'runtime/trim_models';dest=base/'aircraft'/model
    shutil.copytree(ROOT/'aircraft'/model,dest,dirs_exist_ok=True)
    p=dest/'Engine/Tiguar_20x10_CCW.xml';tree=ET.parse(p);el=tree.getroot().find('ixx');el.set('unit','KG*M2');el.text='.05';tree.write(p)
    return base

def trim(base,model,eas,climb=0,bank=0):
    f=jsbsim.FGFDMExec(str(base));f.set_debug_level(0)
    f.set_aircraft_path('aircraft');f.set_engine_path(f'aircraft/{model}/Engine')
    assert f.load_model(model)
    f.disable_output()
    f['ic/h-sl-ft']=1200;f['ic/vc-kts']=eas/.514444
    f['ic/gamma-deg']=np.degrees(np.arcsin(climb/(eas*1.02)))
    f['ic/phi-deg']=bank;f.run_ic()
    row={'model':model,'eas':eas,'climb_requested':climb,'bank_requested':bank}
    try:
        f.do_trim(5 if bank else 0)
        row.update(success=True,throttle=f['fcs/throttle-cmd-norm'],rpm=f['propulsion/engine/propeller-rpm'],
                   alpha_deg=f['aero/alpha-deg'],pitch_deg=f['attitude/theta-deg'],bank_deg=f['attitude/phi-deg'],
                   thrust_n=f['propulsion/engine/thrust-lbs']*4.448221615,
                   climb=-f['velocities/v-down-fps']*.3048,turn_rate=np.degrees(f['velocities/psidot-rad_sec']),
                   udot=f['accelerations/udot-ft_sec2'],wdot=f['accelerations/wdot-ft_sec2'])
    except jsbsim.TrimFailureError:
        row['success']=False
    return row

def main():
    rows=[]
    for model in ['tig','tig_calibrated']:
        base=setup(model)
        for v in [18,20,22,23,24,26,28]:
            rows.append(trim(base,model,v))
        for climb in [-2,-1,1,2,3]:rows.append(trim(base,model,23,climb))
        for bank in [18,-18,30,-30]:rows.append(trim(base,model,23,0,bank))
    df=pd.DataFrame(rows);df.to_csv(ROOT/'analysis/jsbsim_trim_sweep.csv',index=False)
    print(df.to_string(index=False))

if __name__=='__main__':main()
