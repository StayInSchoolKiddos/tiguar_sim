#!/usr/bin/env python3
"""Check physical/model and dataset invariants; fidelity failures stay visible in the report."""
import json
from pathlib import Path
import sys
import unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'.deps'))
import jsbsim
import numpy as np
import pandas as pd
MODEL='tig_calibrated'
RUN_DIR=ROOT/'runtime/candidate_final_scaled'
CALIBRATION=ROOT/'config/calibration.json'

def model():
    f=jsbsim.FGFDMExec(str(ROOT));f.set_debug_level(0)
    output=ROOT/'runtime/pipeline_checks';output.mkdir(parents=True,exist_ok=True)
    f.set_output_path(str(output))
    f.set_aircraft_path('aircraft');f.set_engine_path(f'aircraft/{MODEL}/Engine')
    assert f.load_model(MODEL)
    f.disable_output()
    f['ic/h-sl-ft']=1200;f['ic/vc-kts']=45;f.set_dt(.001);f.run_ic()
    return f

class PipelineChecks(unittest.TestCase):
    def test_loaded_mass_cg_and_inertia(self):
        f=model()
        self.assertAlmostEqual(f['inertia/weight-lbs'],55,places=3)
        self.assertAlmostEqual(f['inertia/cg-x-in']*.0254,.94,places=5)
        self.assertAlmostEqual(f['inertia/cg-z-in']*.0254,0,places=5)
        # JSBSim's XML unit-conversion table uses this rounded constant.
        I=np.array([f[f'inertia/i{axis}{axis}-slugs_ft2']*1.35594 for axis in 'xyz'])
        np.testing.assert_allclose(I,[20.5,15.5,34],rtol=1e-5)
        self.assertTrue(np.all(I>0));self.assertTrue(2*max(I)<sum(I))

    def test_control_signs(self):
        for control,accel,sign in [('elevator','qdot',-1),('aileron','pdot',1)]:
            values=[]
            for value in [-.1,.1]:
                f=model();f[f'fcs/{control}-cmd-norm']=value;f.run()
                values.append(f[f'accelerations/{accel}-rad_sec2'])
            self.assertGreater(sign*(values[1]-values[0]),0)

    def test_data_partition_and_windows(self):
        for date in ['2026-04-30','2026-05-01']:
            df=pd.read_csv(ROOT/f'analysis/{date}/samples.csv.gz')
            self.assertTrue(df.t.is_monotonic_increasing)
            s=pd.read_csv(ROOT/f'analysis/{date}/segments.csv')
            for row in s.itertuples():
                w=df[df.t.between(row.start_s-1e-6,row.end_s+1e-6)]
                self.assertEqual(w.flight.nunique(),1)
                self.assertEqual(w.flight.iloc[0],row.flight)
                self.assertGreaterEqual((w.airspeed_healthy==1).mean(),.98)
                self.assertGreaterEqual((w.flap.abs()<1).mean(),.98)
                self.assertTrue(w.throttle_output_norm.between(0,1).all())
        c=json.loads(CALIBRATION.read_text())
        self.assertNotEqual(c['training'],c['held_out'])
        self.assertTrue(np.all(np.diff(c['shaft_power_w'])>=0))

    def test_coupled_run_completed(self):
        p=RUN_DIR
        summary=json.loads((p/'summary.json').read_text())
        self.assertTrue(summary['valid'])
        s=pd.read_csv(p/'telemetry.csv')
        self.assertTrue(np.isfinite(s[['eas','alt','roll','pitch']]).all().all())
        for seq,target_alt,target_eas in [(3,100,23),(5,100,25),(7,130,23),(8,100,23),(9,100,23)]:
            w=s[s.mission_seq==seq]
            self.assertGreater(len(w),100)
            w=w[w.t>w.t.max()-20]
            self.assertLess(abs(w.alt.mean()-target_alt),3)
            self.assertLess(abs(w.eas.mean()-target_eas),1)
        self.assertGreater(s[s.mission_seq==3].tail(200).roll.mean(),10)
        self.assertLess(s[s.mission_seq==9].tail(200).roll.mean(),-10)

if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser()
    parser.add_argument('--model',default=MODEL)
    parser.add_argument('--run-dir',type=Path,default=RUN_DIR)
    parser.add_argument('--config',type=Path,default=CALIBRATION)
    args,remaining=parser.parse_known_args()
    MODEL=args.model; RUN_DIR=args.run_dir; CALIBRATION=args.config
    unittest.main(argv=[sys.argv[0]]+remaining)
