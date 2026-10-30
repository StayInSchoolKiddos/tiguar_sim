#!/usr/bin/env python3
"""Isolated pitch sensitivity experiments; does not change the candidate or defaults."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
WORK=ROOT/'runtime/da35_pitch_investigation'
MODEL='tig_da35_cruise'
CASES={
    'baseline':({},False),
    'scaling_speed':({'SCALING_SPEED':23},False),
    'rotation_speed':({'TKOFF_ROTATE_SPD':17},False),
    'both_settings':({'SCALING_SPEED':23,'TKOFF_ROTATE_SPD':17},False),
    'unscaled_moments':({},True),
    'both_unscaled':({'SCALING_SPEED':23,'TKOFF_ROTATE_SPD':17},True),
}

def prepare():
    WORK.mkdir(parents=True,exist_ok=True)
    # Freeze the configuration that produced the original plotted overshoot.
    base=(ROOT/'runtime/da35_cruise_validation/sitl.parm').read_text()
    manifest={}
    for case,(overrides,unscale) in CASES.items():
        setup=WORK/'inputs'/case;setup.mkdir(parents=True,exist_ok=True)
        source=setup/'aircraft'/MODEL
        shutil.copytree(ROOT/'aircraft'/MODEL,source,dirs_exist_ok=True)
        if unscale:
            path=source/f'{MODEL}.xml';tree=ET.parse(path)
            for fn in tree.findall("aerodynamics/axis[@name='PITCH']/function"):
                product=fn.find('product');last=product[-1]
                assert last.tag=='value' and abs(float(last.text)-15.5/4.101)<1e-8
                product.remove(last)
            tree.write(path,encoding='utf-8',xml_declaration=True)
        lines=[line for line in base.splitlines() if not line.split() or line.split()[0] not in overrides]
        (setup/'sitl.parm').write_text('\n'.join(lines)+'\n'+''.join(f'{k} {v}\n' for k,v in overrides.items()))
        manifest[case]={'overrides':overrides,'pitch_moment_multiplier':1 if unscale else 15.5/4.101,
            'model_sha256':hashlib.sha256((source/f'{MODEL}.xml').read_bytes()).hexdigest(),
            'parameter_sha256':hashlib.sha256((setup/'sitl.parm').read_bytes()).hexdigest()}
    (WORK/'experiments.json').write_text(json.dumps(manifest,indent=2)+'\n')

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',action='store_true');p.add_argument('--case',choices=CASES);args=p.parse_args()
    prepare()
    if args.run:
        for case in ([args.case] if args.case else CASES):
            setup=WORK/'inputs'/case
            subprocess.run([sys.executable,str(ROOT/'scripts/run_sitl.py'),'--model',MODEL,
                '--name',f'da35_pitch_investigation/{case}','--duration','75','--profile',
                '--param-file',str(setup/'sitl.parm'),'--aircraft-dir',str(setup/'aircraft')],check=True,cwd=ROOT)

if __name__=='__main__':main()
