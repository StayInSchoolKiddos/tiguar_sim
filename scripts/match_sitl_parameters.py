#!/usr/bin/env python3
"""Match flight-control parameters at a BIN timestamp, with explicit SITL exceptions."""
import argparse
import hashlib
import json
from pathlib import Path
from pymavlink import DFReader

ROOT=Path(__file__).resolve().parents[1]
DEFAULT_LOG=ROOT/'2026-05-01 14-00 - 13 flights.BIN'
PREFIXES=('RLL_','RLL2SRV_','PTCH_','PTCH2SRV_','YAW_','YAW2SRV_',
          'TECS_','NAVL1_','AIRSPEED_','TKOFF_','LAND_','KFF_','FLAP_','ACRO_','FBWB_')
EXACT={'SCALING_SPEED','ROLL_LIMIT_DEG','LEVEL_ROLL_LIMIT','STALL_PREVENTION',
       'STICK_MIXING','FLIGHT_OPTIONS','THR_MAX','THR_MIN','THR_SLEWRATE',
       'TRIM_THROTTLE','WP_LOITER_RAD','WP_RADIUS','WP_MAX_RADIUS','ARSPD_USE','RUDD_DT_GAIN',
       'STAB_PITCH_DOWN','RUDDER_ONLY','THROTTLE_NUDGE','THR_PASS_STAB','THR_SUPP_MAN',
       'SERVO_AUTO_TRIM','SCHED_LOOP_RATE','INS_GYRO_FILTER','INS_ACCEL_FILTER',
       'CRUISE_ALT_FLOOR','ALT_OFFSET'}
OVERRIDES={
    'SERVO1_FUNCTION':4,'SERVO2_FUNCTION':19,'SERVO3_FUNCTION':70,'SERVO4_FUNCTION':21,
    'SERVO1_REVERSED':0,'SERVO2_REVERSED':1,'SERVO3_REVERSED':0,'SERVO4_REVERSED':1,
    'LOG_DISARMED':1,'LOG_BITMASK':65535,'SIM_WIND_SPD':0,'SIM_WIND_TURB':0,
    'TERRAIN_ENABLE':0,'ARMING_CHECK':0,'FS_GCS_ENABL':0,'THR_FAILSAFE':0,'FLTMODE_CH':0,
}
for ch in range(1,5):
    OVERRIDES.update({f'SERVO{ch}_MIN':1000,f'SERVO{ch}_MAX':2000,f'SERVO{ch}_TRIM':1000 if ch==3 else 1500})


def snapshot(log,at_time):
    reader=DFReader.DFReader_binary(str(log));values={};times={}
    while (m:=reader.recv_match(type='PARM')) is not None:
        t=m.TimeUS*1e-6
        if t<=at_time:
            values[m.Name]=float(m.Value);times[m.Name]=t
    required={'SCALING_SPEED','TKOFF_ROTATE_SPD','PTCH_RATE_P','PTCH_TRIM_DEG','AIRSPEED_CRUISE'}
    if not required<=values.keys():raise ValueError(f'Missing required parameters before {at_time}: {required-values.keys()}')
    return values,times


def generate(log=DEFAULT_LOG,at_time=1111.,output=ROOT/'config/sitl.parm'):
    log=Path(log).resolve();output=Path(output).resolve()
    values,times=snapshot(log,at_time)
    selected={k:v for k,v in values.items() if k.startswith(PREFIXES) or k in EXACT}
    selected.update(OVERRIDES)
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text('# SIMULATION ONLY: equivalent axes on outputs 1-4; do not load onto hardware.\n'
        f'# Generated from {log.name}, latest PARM at or before {at_time:g} seconds.\n'
        '# Reproduce: python3 scripts/match_sitl_parameters.py\n'
        +''.join(f'{k} {v:.9g}\n' for k,v in sorted(selected.items())))
    audit=[]
    for k in sorted(set(values)|set(selected)):
        action='simulation override' if k in OVERRIDES else 'matched' if k in selected else 'not imported'
        reason=('Equivalent simulator axes, local harness, logging or environment' if k in OVERRIDES
                else 'Flight-control or navigation parameter' if k in selected
                else 'Outside flight-control import scope; simulator defaults remain (hardware, sensors, interfaces, etc.)')
        audit.append({'name':k,'source_value':values.get(k),'source_time_s':times.get(k),
                      'simulator_value':selected.get(k),'action':action,'reason':reason})
    meta={'source':str(log.relative_to(ROOT)) if log.is_relative_to(ROOT) else str(log),
          'source_sha256':hashlib.sha256(log.read_bytes()).hexdigest(),'at_time_s':at_time,
          'output_sha256':hashlib.sha256(output.read_bytes()).hexdigest(),
          'matched_count':sum(r['action']=='matched' for r in audit),'override_count':len(OVERRIDES),
          'limitations':['This snapshot does not replay later parameter changes or mission commands.',
                        'Real ruddervator outputs are replaced by equivalent simulator elevator/rudder axes.',
                        'Flap commands are copied, but aerodynamic flap effects and electric assistance are not modeled.',
                        'Hardware sensor calibration, mounting offsets and device settings are not imported.'],
          'parameters':audit}
    output.with_suffix('.audit.json').write_text(json.dumps(meta,indent=2)+'\n')
    print(f'Wrote {output}: {meta["matched_count"]} matched flight parameters, {len(OVERRIDES)} simulator overrides')
    return meta


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--log',type=Path,default=DEFAULT_LOG)
    p.add_argument('--at-time',type=float,default=1111.);p.add_argument('--output',type=Path,default=ROOT/'config/sitl.parm')
    a=p.parse_args();generate(a.log,a.at_time,a.output)

if __name__=='__main__':main()
