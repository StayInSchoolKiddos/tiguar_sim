#!/usr/bin/env python3
"""Fit a conditional propulsion/drag model, and score a separate flight day.

Mass, geometry and induced drag remain priors: narrow-speed data cannot identify
all of them. Thrust scale is an effective correction, not a thrust measurement.
"""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pandas as pd
from scipy.optimize import least_squares
from scipy.interpolate import PchipInterpolator
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
MASS=55*.45359237; SPAN=3.888; AREA=SPAN*(.3299968+.205359)/2; G=9.80665; D=.508
K=.04

def prop_tables():
    root=ET.parse(ROOT/'aircraft/tig/Engine/Tiguar_20x10_CCW.xml').getroot()
    return [np.array([[float(x) for x in l.split()] for l in root.find(f"table[@name='{n}']/tableData").text.strip().splitlines()]) for n in ['C_THRUST','C_POWER']]

def add_physics(s):
    ct,cp=prop_tables();s=s.copy(); n=s.rpm/60
    s['J']=s.tas/(n*D)
    s['prop_in_range']=s.J.between(0,ct[-1,0])
    s['thrust_prior']=s.rho*n*n*D**4*np.interp(s.J,ct[:,0],ct[:,1])
    s['power_prior']=s.rho*n**3*D**5*np.interp(s.J,cp[:,0],cp[:,1])
    s['qS']=.5*s.rho*s.tas**2*AREA
    s['CL']=MASS*G/(s.qS*np.cos(np.deg2rad(s.roll)))
    s['energy_force']=MASS*(s.accel+G*s.climb/s.tas)
    return s

def main():
    out=ROOT/'analysis';train=add_physics(pd.read_csv(out/'2026-05-01/segments.csv'))
    test=add_physics(pd.read_csv(out/'2026-04-30/segments.csv'))
    fit=train[train.prop_in_range].copy()
    # Two free parameters only. k, mass and geometry are explicit assumptions.
    def residual(x):
        scale,cd0=x
        return (scale*fit.thrust_prior-fit.qS*(cd0+K*fit.CL**2)-fit.energy_force).to_numpy()
    result=least_squares(residual,[1.5,.025],bounds=([.25,.005],[4,.12]),loss='soft_l1',f_scale=4)
    scale,cd0=result.x
    # Monotone power/throttle fit with a low-order nonnegative basis.
    t=fit.throttle_output_norm.to_numpy();powers=fit.power_prior.to_numpy()
    design=np.column_stack([np.ones(len(t)),t,t*t])
    powerfit=least_squares(lambda c:(design@c-powers)/100,[30,1200,600],bounds=(0,np.inf),loss='soft_l1')
    coeff=powerfit.x
    grid=np.linspace(0,1,21);watts=coeff[0]+coeff[1]*grid+coeff[2]*grid**2
    scores={}
    fig,ax=plt.subplots(2,2,figsize=(12,9))
    for name,s,color in [('training',train,'tab:blue'),('held_out',test,'tab:orange')]:
        s=s[s.prop_in_range].copy()
        s['thrust_estimate']=scale*s.thrust_prior
        s['drag_estimate']=s.qS*(cd0+K*s.CL**2)
        s['climb_pred_from_rpm']=(s.thrust_estimate-s.drag_estimate-MASS*s.accel)*s.tas/(MASS*G)
        pfit=coeff[0]+coeff[1]*s.throttle_output_norm+coeff[2]*s.throttle_output_norm**2
        # Predict equilibrium RPM from the fitted throttle-to-shaft-power mapping.
        cp=prop_tables()[1]
        ns=np.linspace(20,160,2000)
        s['rpm_pred']=[60*ns[np.argmin(abs(rho*ns**3*D**5*np.interp(v/(ns*D),cp[:,0],cp[:,1])-p))] for v,rho,p in zip(s.tas,s.rho,pfit)]
        ct=prop_tables()[0];nn=s.rpm_pred/60
        thrust=scale*s.rho*nn**2*D**4*np.interp(s.tas/(nn*D),ct[:,0],ct[:,1])
        s['climb_pred_from_throttle']=(thrust-s.drag_estimate-MASS*s.accel)*s.tas/(MASS*G)
        s.to_csv(out/f'{name}_predictions.csv',index=False)
        scores[name]={'windows':len(s),'rpm_rmse':float(np.sqrt(np.mean((s.rpm_pred-s.rpm)**2))),
                     'climb_rmse_from_rpm':float(np.sqrt(np.mean((s.climb_pred_from_rpm-s.climb)**2))),
                     'climb_rmse_from_throttle':float(np.sqrt(np.mean((s.climb_pred_from_throttle-s.climb)**2))),
                     'airspeed_min':float(s.eas.min()),'airspeed_max':float(s.eas.max())}
        ax[0,0].scatter(s.throttle_output_norm,s.rpm,label=name,c=color)
        ax[0,1].scatter(s.rpm,s.rpm_pred,label=name,c=color)
        ax[1,0].scatter(s.climb,s.climb_pred_from_throttle,label=name,c=color)
        ax[1,1].scatter(s.roll.abs(),s.throttle_output_norm,label=name,c=color)
    ax[0,0].set(xlabel='Actual normalized throttle output',ylabel='Logged RPM')
    ax[0,1].plot([3000,7500],[3000,7500],'k--');ax[0,1].set(xlabel='Logged RPM',ylabel='Predicted RPM')
    ax[1,0].plot([-2,3],[-2,3],'k--');ax[1,0].set(xlabel='Logged climb (m/s)',ylabel='Predicted climb from throttle (m/s)')
    ax[1,1].set(xlabel='Absolute bank (deg)',ylabel='Actual normalized throttle output')
    for a in ax.flat:a.grid(alpha=.25);a.legend()
    fig.suptitle(f'Conditional steady-window fit — approximate mass {MASS:.2f} kg; May 1 training, April 30 held out')
    fig.tight_layout();fig.savefig(ROOT/'reports/performance_fit.png',dpi=150);plt.close(fig)
    config={'mass_kg':MASS,'span_m':SPAN,'area_m2':AREA,'root_chord_m':.3299968,'tip_chord_m':.205359,
            'cd0':float(cd0),'induced_k':K,'thrust_scale':float(scale),'power_coefficients_w':coeff.tolist(),
            'throttle_grid':grid.tolist(),'shaft_power_w':watts.tolist(),'scores':scores,
            'training':'2026-05-01','held_out':'2026-04-30',
            'assumptions':['User estimates takeoff mass at approximately 55 lb; fuel burn not measured.',
            'Area assumes a simple trapezoid; inboard planform not measured.',
            'Induced drag k fixed at 0.04; narrow speed range cannot identify a full drag polar.',
            'Thrust scale is an effective fit conditional on mass/propeller/drag, not measured thrust.',
            'Power outside observed throttle range is extrapolated.',
            'Candidate loaded CG is provisionally x=0.94 m, z=0 aligned with the thrust datum; actual CG unknown.',
            'Candidate loaded inertias 20.5/15.5/34 kg*m2: yaw is a physically admissible prior, not the inconsistent 41.4 measurement.',
            'Different-day propulsion/configuration differences remain possible.']}
    (ROOT/'config/calibration.json').write_text(json.dumps(config,indent=2))
    print(json.dumps(config,indent=2))

if __name__=='__main__':main()
