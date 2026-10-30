#!/usr/bin/env python3
"""Validate the shared BIN-derived configuration on each maintained Tiguar profile."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
import pandas as pd
from match_sitl_parameters import snapshot
from sitl_parameters import default_text, values
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'reports/da35_cruise_analysis'
MODELS={'tig_da35_cruise':'da35_default75_validation','tig_calibrated':'bin_matched_tig_calibrated','tig_da35':'bin_matched_tig_da35'}

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',action='store_true');args=parser.parse_args()
    audit=json.loads((ROOT/'config/sitl.audit.json').read_text());results=[];checks=[]
    OUT.mkdir(parents=True,exist_ok=True)
    for model,name in MODELS.items():
        run=ROOT/'runtime'/name
        if args.run:
            print(f'Running {model} with the shared BIN-matched configuration...',flush=True)
            with (OUT/f'{model}_validation_console.txt').open('w') as log:
                subprocess.run([sys.executable,str(ROOT/'scripts/run_sitl.py'),'--model',model,'--name',name,'--profile','--duration','620'],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True)
        assert (run/'sitl.parm').read_text()==default_text(model)
        expected_values=values(default_text(model))
        model_overrides={p['name']:expected_values[p['name']] for p in audit['parameters']
                         if p['action']!='not imported' and expected_values[p['name']]!=values((ROOT/'config/sitl.parm').read_text())[p['name']]}
        for rel in [f'{model}.xml','Engine/Tiguar_20x10_CCW.xml','Engine/empirical_power.xml' if model=='tig_calibrated' else 'Engine/DA35.xml']:
            assert (run/'aircraft'/model/rel).read_bytes()==(ROOT/'aircraft'/model/rel).read_bytes()
        loaded,_=snapshot(run/'logs/00000001.BIN',60)
        count=0
        for p in audit['parameters']:
            if p['action']=='not imported':continue
            value=loaded.get(p['name']);expected=expected_values[p['name']]
            ok=value is not None and bool(np.isclose(value,expected,rtol=1e-6,atol=1e-7))
            checks.append({'model':model,'name':p['name'],'expected':expected,'loaded':value,'matches':ok,'model_override':p['name'] in model_overrides})
            count+=ok
        check=subprocess.run([sys.executable,str(ROOT/'scripts/check_pipeline.py'),'--model',model,'--run-dir',str(run)],cwd=ROOT,text=True,capture_output=True)
        (OUT/f'{model}_parameter_pipeline_checks.txt').write_text(check.stdout+check.stderr)
        result={'model':model,'run':str(run.relative_to(ROOT)),'parameter_values_verified':count,'model_overrides':model_overrides,
            'all_parameters_match':all(r['matches'] for r in checks if r['model']==model),
            'pipeline_passed':check.returncode==0,'parameters_sha256':hashlib.sha256((run/'sitl.parm').read_bytes()).hexdigest()}
        results.append(result)
        print(json.dumps(result),flush=True)
        (OUT/'parameter_profile_validation.json').write_text(json.dumps(results,indent=2)+'\n')
        pd.DataFrame(checks).to_csv(OUT/'parameter_profile_validation.csv',index=False)
    if not all(r['pipeline_passed'] and r['all_parameters_match'] for r in results):raise SystemExit('Some checks failed; see saved results.')

if __name__=='__main__':main()
