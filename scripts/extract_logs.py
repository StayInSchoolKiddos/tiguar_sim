#!/usr/bin/env python3
"""Extract synchronized flight evidence without assuming hardware channel order.

Writes 10 Hz tables, parameter histories, flight/segment manifests and missions.
No interpolation across stale signals; parameter values are applied as-of time.
"""
import argparse
import json
from pathlib import Path
import numpy as np
import pandas as pd
from pymavlink import DFReader

FIELDS = {
    'CTUN': {'As':'eas','E2T':'eas2tas','ThO':'throttle_pct','NavRoll':'roll_dem','NavPitch':'pitch_dem'},
    'ATT': {'Roll':'roll','Pitch':'pitch','Yaw':'yaw'},
    'ARSP': {'Airspeed':'pitot_eas','H':'airspeed_healthy','U':'airspeed_used'},
    'RPM': {'rpm1':'rpm'},
    'BARO': {'Alt':'baro_alt','Press':'pressure','Temp':'baro_temp'},
    'XKF1': {'VN':'vn','VE':'ve','VD':'vd','PN':'pn','PE':'pe','PD':'pd'},
    'XKF2': {'VWN':'wind_n','VWE':'wind_e'},
    'IMU': {'GyrX':'p','GyrY':'q','GyrZ':'r'},
    'GPS': {'Alt':'gps_alt','Spd':'groundspeed','Status':'gps_status'},
    'STAT': {'isFlying':'is_flying','Armed':'armed','Stage':'stage'},
    'MODE': {'ModeNum':'mode'},
    'AETR': {'Ail':'aileron','Elev':'elevator','Thr':'aetr_throttle','Rudd':'rudder','Flap':'flap'},
    'TECS': {'h':'tecs_alt','dh':'tecs_climb','hdem':'alt_dem','spdem':'speed_dem','th':'tecs_throttle'},
    'RCOU': {f'C{i}':f'pwm{i}' for i in range(1,15)},
    'RCO2': {f'C{i}':f'pwm{i}' for i in range(15,19)},
}

def runs(mask):
    a=np.asarray(mask,dtype=bool)
    edges=np.diff(np.r_[False,a,False].astype(int))
    return zip(np.where(edges==1)[0], np.where(edges==-1)[0])

def extract(path, out):
    out.mkdir(parents=True,exist_ok=True)
    r=DFReader.DFReader_binary(str(path))
    streams={k:[] for k in FIELDS}; params=[]; messages=[]; missions=[]
    selected=list(FIELDS)+['PARM','MSG','CMD']
    while (m:=r.recv_match(type=selected)) is not None:
        typ=m.get_type(); d=m.to_dict(); t=d.get('TimeUS',0)*1e-6
        if typ=='PARM': params.append({'t':t,'name':m.Name,'value':m.Value});continue
        if typ=='MSG': messages.append({'t':t,'message':m.Message});continue
        if typ=='CMD': missions.append(d);continue
        if d.get('I',d.get('C',0))!=0:continue
        streams[typ].append({'t':t,**{v:d.get(k,np.nan) for k,v in FIELDS[typ].items()}})
    ct=pd.DataFrame(streams['CTUN'])
    df=pd.DataFrame({'t':np.arange(np.ceil(ct.t.min()*10)/10,ct.t.max(),.1)})
    for typ,rows in streams.items():
        if not rows:
            for col in FIELDS[typ].values():df[col]=np.nan
            continue
        s=pd.DataFrame(rows).sort_values('t').drop_duplicates('t',keep='last')
        df=pd.merge_asof(df,s,on='t',direction='backward',tolerance=None if typ=='MODE' else .6)
    ph=pd.DataFrame(params)
    ph.to_csv(out/'parameters.csv',index=False)
    # Apply output assignment, endpoints, and reversals at each sample's timestamp.
    df['throttle_pwm']=np.nan;df['throttle_output_norm']=np.nan;df['throttle_channel']=np.nan
    for ch in range(1,19):
        names=[f'SERVO{ch}_{suffix}' for suffix in ['FUNCTION','MIN','MAX','REVERSED']]
        if not set(names)<=set(ph.name):continue
        pv={}
        for name in names:
            series=ph[ph.name==name][['t','value']].sort_values('t').drop_duplicates('t',keep='last')
            pv[name.rsplit('_',1)[1]]=pd.merge_asof(df[['t']],series,on='t',direction='backward')['value']
        active=pv['FUNCTION']==70
        norm=(df[f'pwm{ch}']-pv['MIN'])/(pv['MAX']-pv['MIN'])
        norm=norm.where(pv['REVERSED']==0,1-norm)
        df.loc[active,'throttle_pwm']=df.loc[active,f'pwm{ch}']
        df.loc[active,'throttle_output_norm']=norm[active]
        df.loc[active,'throttle_channel']=ch
    df['tas']=df.eas*df.eas2tas
    # ArduPilot EAS2TAS supplies the density conversion; avoid hot barometer temperature as ambient.
    df['rho']=1.225/df.eas2tas**2
    df['climb']=-df.vd
    df['altitude']=-df.pd
    for name in ['eas','tas','roll','pitch','altitude','climb','rpm','throttle_output_norm']:
        df[name+'_smooth']=df[name].rolling(21,center=True,min_periods=15).mean()
    df['accel']=df.tas_smooth.diff()/df.t.diff()
    df['turn_rate']=np.rad2deg((df.q*np.sin(np.deg2rad(df.roll))+df.r*np.cos(np.deg2rad(df.roll)))/np.cos(np.deg2rad(df.pitch)))
    df['flight']=0
    flight_rows=[];n=0
    # STAT hysteresis is the primary detector, with speed to remove ground false positives.
    airborne=(df.is_flying==1)&(df.eas>12)&(df.gps_status>=3)
    # Bridge brief detector/sensor gaps, at most two seconds.
    for a,b in list(runs(~airborne)):
        if a>0 and b<len(df) and b-a<=20:airborne.iloc[a:b]=True
    for a,b in runs(airborne):
        if b-a<150:continue
        n+=1;df.loc[a:b-1,'flight']=n
        flight_rows.append({'flight':n,'start_s':df.t.iloc[a],'end_s':df.t.iloc[b-1],'duration_s':(b-a)/10,'max_eas':df.eas.iloc[a:b].max()})
    from segments import select
    segments=select(df).to_dict("records")
    df.to_csv(out/'samples.csv.gz',index=False)
    pd.DataFrame(flight_rows).to_csv(out/'flights.csv',index=False)
    pd.DataFrame(segments).to_csv(out/'segments.csv',index=False)
    final=ph.groupby('name',sort=False).tail(1).set_index('name').value.to_dict()
    (out/'parameters_final.json').write_text(json.dumps(final,indent=2))
    (out/'metadata.json').write_text(json.dumps({'source':str(path),'messages':messages,'missions':missions,'sample_rows':len(df),'flights':len(flight_rows),'segments':len(segments),'stream_counts':{k:len(v) for k,v in streams.items()}},indent=2))
    print(path.name, 'flights',n,'segments',len(segments),'rows',len(df),flush=True)

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('logs',nargs='+',type=Path);args=ap.parse_args()
    for p in args.logs:extract(p,Path('analysis')/p.name[:10])
