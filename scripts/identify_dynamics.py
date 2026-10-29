#!/usr/bin/env python3
"""Exploratory effective control derivatives, conditional on seed inertias.

These regressions use estimated alpha and normalized actuator commands; they do
not resolve physical inertia or servo travel. They are diagnostics, not accepted
airframe coefficients, until short-horizon simulation validation passes.
"""
import json
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.signal import savgol_filter
from scipy.optimize import least_squares
ROOT=Path(__file__).resolve().parents[1]

def features(date):
    d=pd.read_csv(ROOT/f'analysis/{date}/samples.csv.gz')
    d['qdot']=savgol_filter(d.q.interpolate().bfill(),11,2,deriv=1,delta=.1)
    d['pdot']=savgol_filter(d.p.interpolate().bfill(),11,2,deriv=1,delta=.1)
    d['alpha']=np.deg2rad(d.pitch)-np.arcsin((d.climb/d.tas).clip(-1,1))
    d['de']=np.where(d.elevator>0,-.35*d.elevator/4500,-.30*d.elevator/4500)
    d['da']=.35*d.aileron/4500
    d['qhat']=d.q*.273/(2*d.tas);d['phat']=d.p*3.888/(2*d.tas)
    good=(d.flight>0)&(d.flap<1)&d.eas.between(20,27)&(d.roll.abs()<10)&(d.airspeed_healthy==1)&(d.q.abs()<.4)&(d.p.abs()<.4)
    d=d[good].dropna(subset=['alpha','de','da','qhat','phat','qdot','pdot','rho','tas']).iloc[::5].copy()
    qS=.5*d.rho*d.tas**2*1.0407316752
    # Effective coefficients include unmodeled CG/thrust effects in the fitted intercept.
    d['pitch_target']=4.101*d.qdot/(qS*.273)
    d['roll_target']=4.643*d.pdot/(qS*3.888)
    return d

def main():
    a=features('2026-05-01');b=features('2026-04-30');results={}
    for axis,columns,start,lower,upper in [
        ('pitch',['alpha','qhat','de'],[.02,-.37,-9,-.5],[-1,-10,-100,-20],[1,0,0,0]),
        ('roll',['phat','da'],[0,-.25,.2],[-1,-10,0],[1,0,10])]:
        X=np.column_stack([np.ones(len(a)),a[columns]]);y=a[axis+'_target'].to_numpy()
        fit=least_squares(lambda c:(X@c-y),start,bounds=(lower,upper),loss='soft_l1',f_scale=.01)
        Xv=np.column_stack([np.ones(len(b)),b[columns]]);yv=b[axis+'_target'].to_numpy()
        pred=Xv@fit.x
        results[axis]={'columns':['intercept']+columns,'coefficients':fit.x.tolist(),'training_rows':len(a),'held_out_rows':len(b),
                       'held_out_rmse':float(np.sqrt(np.mean((pred-yv)**2))),
                       'held_out_zero_prediction_rmse':float(np.sqrt(np.mean(yv**2))),
                       'status':'diagnostic only; closed-loop identification and alpha/actuator uncertainties'}
    (ROOT/'analysis/dynamics_diagnostic.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))

if __name__=='__main__':main()
