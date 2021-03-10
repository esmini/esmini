import re
from test_common import *
import unittest

ESMINI_PATH = '../'
COMMON_ARGS = '--headless --fixed_timestep 0.01 --record sim.dat '


class TestSuite(unittest.TestCase):
    
    def test_cut_in(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/cut-in.xosc'), COMMON_ARGS)
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading ../resources/xosc/cut-in.xosc', log))
        
        # Check some scenario events
        self.assertTrue(re.search('0.010 .* CutInActStart == true, 0.0100 > 0.00 edge: NONE', log))
        self.assertTrue(re.search('\n[789].* BrakeCondition == true, HWT: 0.70 > 0.70, edge Rising', log))
        self.assertTrue(re.search('\n22.* ActStopCondition timer expired at 5.00 seconds', log))
        

    def test_ltap_od(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/ltap-od.xosc'), COMMON_ARGS \
            + '--disable_controllers')
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading ../resources/xosc/ltap-od.xosc', log))
        self.assertTrue(re.search('.*Route::AddWaypoint Added connecting waypoint 1: 9, -1, 0.00', log))

        # Check some scenario events
        self.assertTrue(re.search('\n5.5.*Synchronize dist \(0.95\) < tolerance \(1.00\)', log))
        self.assertTrue(re.search('\n9.5.* QuitCondition timer expired at 4.0. seconds', log))

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('\n6.500, 0, Ego, 28.5., -7.8., 0.00, 1.[78].', csv))
        self.assertTrue(re.search('\n6.500, 1, NPC, 24.5., 0.2., 0.00, 5.[34].', csv))


if __name__ == "__main__":
    # execute only if run as a script

    unittest.main(verbosity=2)
