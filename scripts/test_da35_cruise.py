"""Check that cruise tuning changes the linkage without changing engine physics."""
import json
import unittest
import xml.etree.ElementTree as ET
import numpy as np
from tune_da35_cruise import ROOT,CONFIG,MODEL,effective_throttle,state
from cruise_calibration import settle


class CruiseTuningChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config=json.loads(CONFIG.read_text())

    def test_engine_propeller_and_airframe_preserved(self):
        for name in ['DA35.xml','Tiguar_20x10_CCW.xml']:
            self.assertEqual((ROOT/'aircraft/tig_da35/Engine'/name).read_bytes(),(ROOT/'aircraft'/MODEL/'Engine'/name).read_bytes())
        before=ET.parse(ROOT/'aircraft/tig_da35/tig_da35.xml').getroot()
        after=ET.parse(ROOT/'aircraft'/MODEL/f'{MODEL}.xml').getroot()
        for tag in ['metrics','mass_balance','ground_reactions','aerodynamics','propulsion']:
            self.assertEqual(ET.tostring(before.find(tag)),ET.tostring(after.find(tag)))

    def test_xml_mapping_is_ordered_and_preserves_endpoints(self):
        root=ET.parse(ROOT/'aircraft'/MODEL/f'{MODEL}.xml').getroot()
        text=root.find("flight_control/channel[@name='DA35 carburetor and starting']/fcs_function[@name='fcs/da35-carburetor']/function/table/tableData").text
        table=np.array([[float(x) for x in line.split()] for line in text.strip().splitlines()])
        self.assertTrue(np.all(np.diff(table[:,0])>0))
        self.assertTrue(np.all(np.diff(table[:,1])>=0))
        self.assertTrue(np.all((table>=0)&(table<=1)))
        for scale in [.75,self.config['scale'],1.15]:
            u=np.linspace(0,1,10001)
            self.assertTrue(np.all(np.diff(effective_throttle(u,scale))>0))
            np.testing.assert_allclose(effective_throttle(np.array([0,.05,.1,.7,.9,1]),scale),[0,.05,.1,.7,.9,1])

    def test_simulated_response_matches_fitted_transform(self):
        # Compare separate FDM instances; validates the generated interpolated XML.
        for throttle in [.125,.255,.45,.6,.9]:
            before=state('tig_da35',23,2449.6953);after=state(MODEL,23,2449.6953)
            rpm_before,ok_before,_=settle(before,float(effective_throttle(throttle,self.config['scale'])))
            rpm_after,ok_after,_=settle(after,throttle)
            self.assertTrue(ok_before and ok_after)
            self.assertLess(abs(rpm_before-rpm_after),5)


if __name__=='__main__':unittest.main()
