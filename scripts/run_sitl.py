#!/usr/bin/env python3
"""Run isolated, pinned ArduPlane with JSBSim and a deterministic takeoff/loiter mission.

This is a software-only harness: local UDP endpoint, no physical autopilot connection.
"""
import argparse
import json
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import time
import pandas as pd
from pymavlink import mavutil

ROOT=Path(__file__).resolve().parents[1]
LAT=38.6961834;LON=-94.2578576

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--model',default='tig_calibrated');ap.add_argument('--duration',type=float,default=240);ap.add_argument('--name');ap.add_argument('--speedup',type=int,default=8);ap.add_argument('--profile',action='store_true');ap.add_argument('--interactive',action='store_true');args=ap.parse_args()
    name=args.name or args.model
    work=ROOT/'runtime'/name;work.mkdir(parents=True,exist_ok=True)
    shutil.copytree(ROOT/'aircraft'/args.model,work/'aircraft'/args.model,dirs_exist_ok=True)
    shutil.copy2(ROOT/'config/sitl.parm',work/'sitl.parm')
    env=os.environ.copy();env['PATH']=str(ROOT/'vendor/jsbsim/build/src')+os.pathsep+env['PATH'];env['JSBSIM_DEBUG']='0'
    defaults=str(ROOT/'vendor/ardupilot/Tools/autotest/default_params/plane-jsbsim.parm')+','+str(work/'sitl.parm')
    # Fail early on stale JSBSim listeners: otherwise ArduPilot can connect to the wrong model.
    with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as check:
        check.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        check.bind(('127.0.0.1',5505))
    command=[str(ROOT/'vendor/ardupilot/build/sitl/bin/arduplane'),'-w','--model','jsbsim:'+args.model,'--home',f'{LAT},{LON},273,0','--speedup',str(args.speedup),'--defaults',defaults,'--autotest-dir',str(work),'--serial0','udpclient:127.0.0.1:14560']
    (work/'command.json').write_text(json.dumps(command,indent=2))
    conn=mavutil.mavlink_connection('udpin:127.0.0.1:14560',source_system=255)
    log=(work/'console.log').open('w');proc=subprocess.Popen(command,cwd=work,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    events=[];rows=[];state={};started=None;ready=False;uploaded=False;mission_sent=False;last_hb=0;simtime=0
    # Home, takeoff, straight climb, indefinite right loiter (~18 degrees at 23 m/s).
    items=[(16,LAT,LON,0,0,0,0),(22,LAT+.004,LON,100,12,0,0),(16,LAT+.009,LON,100,0,0,0),(17,LAT+.009,LON,100,0,0,170)]
    if args.profile:
        items=items[:3]+[(19,LAT+.009,LON,100,90,0,170),(178,0,0,0,0,25,-1),
            (19,LAT+.009,LON,100,60,0,170),(178,0,0,0,0,23,-1),
            (19,LAT+.009,LON,130,90,0,170),(19,LAT+.009,LON,100,90,0,170),
            (19,LAT+.009,LON,100,90,0,-170),(17,LAT+.009,LON,100,0,0,-170)]
    (work/'mission.json').write_text(json.dumps(items,indent=2))
    def cmd(command,*params):
        conn.mav.command_long_send(1,1,command,0,*(list(params)+[0]*(7-len(params))))
    wall=time.monotonic();start_wall=wall;children=set()
    try:
        while args.interactive or time.monotonic()-start_wall<max(240,args.duration):
            childfile=Path(f'/proc/{proc.pid}/task/{proc.pid}/children')
            if childfile.exists():children.update(childfile.read_text().split())
            if proc.poll() is not None:raise RuntimeError(f'ArduPlane exited {proc.returncode}; see {work}/console.log')
            now=time.monotonic()
            if now-last_hb>1:
                conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0);last_hb=now
            msg=conn.recv_match(blocking=True,timeout=.1)
            if msg is None:continue
            d=msg.to_dict();typ=msg.get_type()
            if typ=='STATUSTEXT':
                events.append({'t':simtime,'text':msg.text});print(f'{name} {simtime:.1f}: {msg.text}',flush=True)
            if typ=='HEARTBEAT':
                state['mode']=msg.custom_mode;state['armed']=int(bool(msg.base_mode&128))
                if not ready:
                    ready=True
                    conn.mav.request_data_stream_send(1,1,mavutil.mavlink.MAV_DATA_STREAM_ALL,10,1)
            if ready and not mission_sent and simtime>25:
                conn.mav.mission_count_send(1,1,len(items));mission_sent=True
            if typ in ['MISSION_REQUEST','MISSION_REQUEST_INT']:
                seq=msg.seq;code,lat,lon,alt,p1,p2,p3=items[seq]
                conn.mav.mission_item_int_send(1,1,seq,3,code,int(seq==0),1,p1,p2,p3,0,int(lat*1e7),int(lon*1e7),alt)
            if typ=='MISSION_ACK':
                uploaded=msg.type==0
                events.append({'t':simtime,'text':f'Mission ACK {msg.type}'})
                if not uploaded:raise RuntimeError(f'Mission upload rejected: {msg.type}')
            if hasattr(msg,'time_boot_ms'):simtime=msg.time_boot_ms/1000
            if uploaded and started is None and simtime>20:
                conn.mav.set_mode_send(1,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,10)
                cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,1,21196)
                cmd(mavutil.mavlink.MAV_CMD_MISSION_START,0,0)
                started=simtime;events.append({'t':simtime,'text':'Harness: AUTO/arm/mission start requested'})
            if typ=='ATTITUDE':state.update(roll=math.degrees(msg.roll),pitch=math.degrees(msg.pitch),yaw=math.degrees(msg.yaw),p=msg.rollspeed,q=msg.pitchspeed,r=msg.yawspeed)
            if typ=='GLOBAL_POSITION_INT':state.update(alt=msg.relative_alt/1000,lat=msg.lat/1e7,lon=msg.lon/1e7,climb=-msg.vz/100)
            if typ=='MISSION_CURRENT':state['mission_seq']=msg.seq
            if typ=='RPM':state['rpm']=msg.rpm1
            if typ=='VFR_HUD':
                state.update(eas=msg.airspeed,groundspeed=msg.groundspeed,throttle_pct=msg.throttle)
                rows.append({'t':simtime,**state})
            if not args.interactive and started is not None and simtime-started>=args.duration:break
        if not rows:raise RuntimeError('No flight telemetry received')
        if not args.interactive and (started is None or simtime-started<args.duration-.2):raise RuntimeError('Simulation timed out before completing the requested duration')
    except KeyboardInterrupt:
        events.append({'t':simtime,'text':'Harness stopped by user'})
    finally:
        # JSBSim creates its own session; stop only descendants of this harness process.
        for pid in children:
            try:os.kill(int(pid),signal.SIGTERM)
            except ProcessLookupError:pass
        if proc.poll() is None:
            proc.terminate()
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
        conn.close();log.close()
        pd.DataFrame(rows).to_csv(work/'telemetry.csv',index=False)
        (work/'events.json').write_text(json.dumps(events,indent=2))
    console=(work/'console.log').read_text()
    if 'Could not bind' in console:raise RuntimeError('Invalid run: JSBSim socket collision')
    df=pd.DataFrame(rows);tail=df[df.t>df.t.max()-60]
    summary={'model':args.model,'sim_end_s':simtime,'wall_duration_s':time.monotonic()-wall,
             'valid':True,'profile':args.profile,
             'max_alt_m':float(df.alt.max()),'last60_mean':tail.select_dtypes('number').mean().to_dict(),
             'last60_std':tail.select_dtypes('number').std().to_dict()}
    (work/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
