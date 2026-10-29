#!/usr/bin/env python3
"""Select nonoverlapping ten-second windows; keep quality metrics for review."""
from pathlib import Path
import json
import numpy as np
import pandas as pd

def select(df):
    rows=[]
    for flight,s in df[df.flight>0].groupby('flight'):
        for a in range(0,len(s)-99,100):
            w=s.iloc[a:a+100]
            if w.t.iloc[-1]-w.t.iloc[0]>10.2:continue
            good=(w.airspeed_healthy==1)&w.eas.between(18,30)&w.rpm.between(1500,9000)&(w.flap.abs()<1)
            if good.mean()<.98:continue
            row={'flight':int(flight),'start_s':w.t.iloc[0],'end_s':w.t.iloc[-1],'duration_s':10}
            for c in ['eas','tas','rho','rpm','throttle_output_norm','roll','pitch','climb','accel','turn_rate','flap','p','q','r']:
                row[c]=w[c].mean();row[c+'_std']=w[c].std()
            if abs(row['accel'])>.35 or row['rpm_std']>700 or row['throttle_output_norm_std']>.15:continue
            if abs(row['roll'])<8 and row['roll_std']<4:
                row['kind']='level' if abs(row['climb'])<.5 else ('climb' if row['climb']>0 else 'descent')
            elif 15<abs(row['roll'])<45 and row['roll_std']<5 and abs(row['climb'])<.8:
                row['kind']='turn'
            else:continue
            rows.append(row)
    return pd.DataFrame(rows)

if __name__=='__main__':
    for path in sorted(Path('analysis').glob('*/samples.csv.gz')):
        s=select(pd.read_csv(path));s.to_csv(path.parent/'segments.csv',index=False)
        meta=path.parent/'metadata.json'
        if meta.exists():
            d=json.loads(meta.read_text());d['segments']=len(s);meta.write_text(json.dumps(d,indent=2))
        print(path.parent.name,s.groupby('kind').size().to_dict() if len(s) else {})
