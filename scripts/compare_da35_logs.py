#!/usr/bin/env python3
"""Score fixed DA35/equivalent engines on recorded steady windows, without refitting."""
import json
import numpy as np
import pandas as pd
from scipy.optimize import brentq
from da35_calibration import ROOT, OUT, MODEL, engine_bench, point
from cruise_calibration import bench, settle, RHO_CONVERSION
from calibrate import add_physics, MASS, G, K
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    config=json.loads((ROOT/'config/calibration.json').read_text())
    rows=[]
    for day in ['2026-04-30','2026-05-01']:
        windows=add_physics(pd.read_csv(ROOT/f'analysis/{day}/segments.csv'))
        for row in windows[windows.prop_in_range].itertuples():
            altitude=brentq(lambda h:bench('tig_calibrated',row.tas,h)['atmosphere/rho-slugs_ft3']*RHO_CONVERSION-row.rho,-1000,20000)
            for model in ['tig_calibrated',MODEL]:
                f=engine_bench(row.tas,altitude) if model==MODEL else bench(model,row.tas,altitude)
                rpm,converged,seconds=settle(f,row.throttle_output_norm)
                thrust=f['propulsion/engine/thrust-lbs']*4.448221615
                j=row.tas/(rpm/60*.508) if rpm>0 else float('inf')
                drag=row.qS*(config['cd0']+K*row.CL**2)
                climb=(thrust-drag-MASS*row.accel)*row.tas/(MASS*G)
                rows.append(dict(date=day,start_s=row.start_s,model=model,tas=row.tas,
                    throttle=row.throttle_output_norm,rpm_observed=row.rpm,rpm_predicted=rpm,
                    climb_observed=row.climb,climb_predicted=climb,converged=converged,
                    prediction_supported=bool(0<=j<=.6674),advance_ratio=j))
    data=pd.DataFrame(rows);data.to_csv(OUT/'steady_log_predictions.csv',index=False)
    eligible=data.groupby(['date','start_s']).apply(lambda x: bool((x.converged & x.prediction_supported).all()))
    data['common_supported']=[eligible.loc[(r.date,r.start_s)] for r in data.itertuples()]
    metrics=[]
    fig,axes=plt.subplots(1,2,figsize=(12,5))
    for (day,model), group in data[data.common_supported].groupby(['date','model']):
        e=group.rpm_predicted-group.rpm_observed
        ce=group.climb_predicted-group.climb_observed
        metrics.append(dict(date=day,model=model,windows=len(group),rpm_rmse=float(np.sqrt(np.mean(e**2))),
            rpm_bias=float(e.mean()),climb_rmse_mps=float(np.sqrt(np.mean(ce**2)))))
        axes[0].scatter(group.rpm_observed,group.rpm_predicted,label=f'{day} {model}',s=20)
        axes[1].scatter(group.climb_observed,group.climb_predicted,label=f'{day} {model}',s=20)
    axes[0].plot([3000,7500],[3000,7500],'k:');axes[0].set(xlabel='Logged RPM',ylabel='Steady predicted RPM')
    axes[1].plot([-2,3],[-2,3],'k:');axes[1].set(xlabel='Logged climb (m/s)',ylabel='Energy-balance predicted climb (m/s)')
    for ax in axes:ax.legend(fontsize=7);ax.grid(alpha=.25)
    fig.tight_layout();fig.savefig(OUT/'steady_log_comparison.png',dpi=150);plt.close(fig)
    frame=pd.DataFrame(metrics);frame.to_csv(OUT/'steady_log_metrics.csv',index=False)
    (OUT/'steady_logs.html').write_text('<!doctype html><html lang="en"><meta charset="utf-8"><title>DA35 comparison with flight logs</title><style>body{font:16px system-ui;max-width:1100px;margin:40px auto}img{width:100%}td,th{padding:8px}</style><h1>DA35 comparison with recorded steady flight</h1><p>Fixed models scored at each recorded throttle, true airspeed and density. No parameters were refitted. Both models use the same supported windows. April 30 was held out from the original log fit; overlap between these logs and the cruise spreadsheet is unknown.</p>'+frame.to_html(index=False,float_format=lambda x:f'{x:.2f}')+'<img src="steady_log_comparison.png"><p>Climb is inferred from modeled thrust and the existing drag/energy balance with recorded acceleration and bank. This is not a coupled flight replay. Propeller coefficients, windmilling, electric assistance and aircraft configuration remain uncertain.</p><p><a href="index.html">DA35 report</a> · <a href="steady_log_predictions.csv">Per-window data</a></p></html>')
    print(frame.to_string(index=False))

if __name__=='__main__':main()
