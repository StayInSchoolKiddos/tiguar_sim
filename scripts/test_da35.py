"""Integration tests for the DA35 candidate; no network or ArduPilot required."""
import json
import unittest
import xml.etree.ElementTree as ET
import numpy as np
from da35_calibration import ROOT, CONFIG, DEST, MODEL, engine_bench, point


class DA35Checks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config=json.loads(CONFIG.read_text())
        cls.altitude=cls.config['reference_altitude_ft']

    def test_native_engine_and_preserved_airframe(self):
        candidate=ET.parse(DEST/f'{MODEL}.xml').getroot()
        baseline=ET.parse(ROOT/'aircraft/tig_calibrated/tig_calibrated.xml').getroot()
        self.assertEqual(candidate.find('propulsion/engine').get('file'),'DA35')
        self.assertEqual(baseline.find('propulsion/engine').get('file'),'empirical_power')
        for section in ['metrics','mass_balance','aerodynamics','ground_reactions']:
            self.assertEqual(ET.tostring(candidate.find(section)),ET.tostring(baseline.find(section)))
        self.assertEqual((DEST/'Engine/Tiguar_20x10_CCW.xml').read_bytes(),
                         (ROOT/'aircraft/tig_calibrated/Engine/Tiguar_20x10_CCW.xml').read_bytes())
        engine=ET.parse(DEST/'Engine/DA35.xml').getroot()
        self.assertEqual(engine.tag,'piston_engine')
        self.assertEqual(len(engine.findall('table')),0)
        self.assertEqual(len(engine.findall('function')),0)

    def test_cold_start_idle_and_shutdown(self):
        f=engine_bench(0,self.altitude,initialized=False)
        idle=point(f,0)
        self.assertTrue(idle['running']);self.assertTrue(idle['converged'])
        self.assertLess(abs(idle['rpm']-1650),50)
        f['propulsion/da35-ignition']=0
        stopped=point(f,0)
        self.assertFalse(stopped['running']);self.assertLess(stopped['rpm'],1)

    def test_cruise_response_and_fuel_consumption(self):
        f=engine_bench(23,self.altitude)
        low=point(f,.225);middle=point(f,.45);high=point(f,.7);again=point(f,.45)
        self.assertLess(abs(middle['rpm']/6263.5-1),.05)
        self.assertLess(low['rpm'],middle['rpm']);self.assertLess(middle['rpm'],high['rpm'])
        self.assertLess(abs(again['rpm']/middle['rpm']-1),.01)
        self.assertTrue(all(p['running'] and p['converged'] for p in [low,middle,high,again]))
        self.assertGreater(middle['fuel_flow_kg_s'],0)
        self.assertLess(again['fuel_kg'],low['fuel_kg'])

    def test_fuel_starvation_stops_combustion(self):
        f=engine_bench(23,self.altitude);point(f,.45)
        f['propulsion/tank/contents-lbs']=0
        result=point(f,.45)
        self.assertFalse(result['running']);self.assertEqual(result['fuel_flow_kg_s'],0)


if __name__=='__main__':unittest.main()
