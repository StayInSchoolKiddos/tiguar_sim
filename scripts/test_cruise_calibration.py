"""Data and calibration checks independent of the coupled-flight acceptance gate."""
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from cruise_calibration import ROOT, read_targets, fit_cruise


class CruiseChecks(unittest.TestCase):
    def test_fleet_mean_and_midpoints(self):
        data = read_targets(ROOT/'rpm_cruise_by_aircraft.csv')
        self.assertAlmostEqual(data.iloc[0].rpm_mean, 2281.25)
        self.assertAlmostEqual(data.iloc[-1].rpm_mean, 7111.75)
        self.assertEqual(data.iloc[-1].throttle, .9)

    def test_reject_invalid_input(self):
        source = (ROOT/'rpm_cruise_by_aircraft.csv').read_text()
        bad_inputs = [source.replace('1856.0', 'NaN'), source.replace('1856.0', '-1'),
                      source.replace('1-5,', '0-5,'), source.replace('80-100,', '80-101,'),
                      source.replace('0-1,', 'bad,'), source.replace('green', 'unknown')]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'input.csv'
            for invalid in bad_inputs:
                path.write_text(invalid)
                with self.subTest(invalid=invalid[:100]), self.assertRaises(ValueError):
                    read_targets(path)

    def test_fit_domain_and_endpoints(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/'fit.json'
            config, targets = fit_cruise(ROOT/'rpm_cruise_by_aircraft.csv', output)
            self.assertFalse(targets.iloc[0].supported)
            self.assertTrue(targets.iloc[-1].supported)
            self.assertTrue(np.all(np.diff(config['throttle_grid']) > 0))
            self.assertTrue(np.all(np.diff(config['shaft_power_w']) >= 0))
            self.assertEqual(config['shaft_power_w'][0], 0)
            self.assertEqual(config['shaft_power_w'][-1], config['shaft_power_w'][-2])
            self.assertNotIn('power_coefficients_w', config)
            self.assertNotIn('scores', config)
            self.assertEqual(json.loads(output.read_text())['cruise']['status'], 'unvalidated')


if __name__ == '__main__':
    unittest.main()
