"""Resolve shared BIN-derived settings plus explicit per-model defaults."""
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def values(text):
    result={}
    for line in text.splitlines():
        fields=line.split()
        if fields and not fields[0].startswith('#'):
            result[fields[0]]=float(fields[1])
    return result

def default_text(model):
    base=(ROOT/'config/sitl.parm').read_text()
    overlay=ROOT/'config/models'/f'{model}.parm'
    if not overlay.exists():return base
    changed=values(overlay.read_text())
    lines=[line for line in base.splitlines() if not line.split() or line.split()[0] not in changed]
    return '\n'.join(lines)+'\n'+overlay.read_text()
