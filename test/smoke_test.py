import re
from test_common import *
import unittest

ESMINI_PATH = '../'
COMMON_ARGS = '--headless --fixed_timestep 0.01 --record sim.dat '


class TestSuite(unittest.TestCase):

    def test_cut_in(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/cut-in.xosc'), COMMON_ARGS)
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading cut-in.xosc', log))
        
        # Check some scenario events
        self.assertTrue(re.search('0.010.* CutInActStart == true, 0.0100 > 0.00 edge: NONE', log))
        self.assertTrue(re.search('\n[789].* BrakeCondition == true, HWT: 0.70 > 0.70, edge Rising', log))
        self.assertTrue(re.search('\n21.[678].* ActStopCondition timer expired at 5.00 seconds', log))

    def test_ltap_od(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/ltap-od.xosc'), COMMON_ARGS \
            + '--disable_controllers')
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading ltap-od.xosc', log))
        self.assertTrue(re.search('.*Route::AddWaypoint Added connecting waypoint 1: 9, -1, 0.00', log))

        # Check some scenario events
        self.assertTrue(re.search('\n5.5.*Synchronize dist \(0.95\) < tolerance \(1.00\)', log))
        self.assertTrue(re.search('\n9.5.* QuitCondition timer expired at 4.0. seconds', log))

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('\n6.500, 0, Ego, 28.5., -7.8., 0.00, 1.[78].', csv))
        self.assertTrue(re.search('\n6.500, 1, NPC, 23.9., 1.0., 0.00, 5.3.', csv))

    def test_trajectory(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/trajectory-test.xosc'), COMMON_ARGS \
            + '--disable_controllers')
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading trajectory-test.xosc', log))

        # Check some scenario events
        self.assertTrue(re.search('\n8.0.*FollowTrajectoryClothoidTrigger == true, element: FollowTrajectoryPLineEvent state: END_TRANSITION', log))
        self.assertTrue(re.search('\n22.99.* FollowTrajectoryNurbsAction runningState -> endTransition -> completeState', log))

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('\n4.100, 0, Ego, 115.04, 4.86, -3.01, 0.28, 0.03', csv))
        self.assertTrue(re.search('\n4.100, 1, Target, 129.91, 14.33, -3.47, 0.50', csv))
        self.assertTrue(re.search('\n11.100, 0, Ego, 200.70, 72.59, -2.44, 1.06, 6.26', csv))
        self.assertTrue(re.search('\n11.100, 1, Target, 205.90, 66.44, -2.50, 2.51', csv))
        self.assertTrue(re.search('\n17.390, 0, Ego, 216.96, 169.84, 2.15, 1.75, 6.21', csv))
        self.assertTrue(re.search('\n17.390, 1, Target, 213.18, 207.84, 4.29, 1.77, 6.25', csv))

    def test_synchronize(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/synchronize.xosc'), COMMON_ARGS \
            + '--disable_controllers')
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading synchronize.xosc', log))

        # Check some scenario events
        self.assertTrue(re.search('\n9.9.* Synchronize dist \(0.92\) < tolerance \(1.00\)', log))
        self.assertTrue(re.search('\n9.9.* Synchronize_NPC_Event complete after 1 execution', log))
        self.assertTrue(re.search('\n19.75.* Free_Speed_Condition_NPC == true, distance 4.81 < tolerance \(5.00\), edge: Rising', log))
        self.assertTrue(re.search('\n19.75.* Triggering entity 0: Ego', log))
        self.assertTrue(re.search('\n28.4.* All acts are done, quit now', log))
        
        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('\n10.000, 0, Ego, 10.20, 299.9., -0.53, 1.56, 0.00, 0.00, 20.00', csv))
        self.assertTrue(re.search('\n10.000, 1, NPC1, 6.70, 305.0., -0.54, 1.56, 0.00, 0.00, 20.00', csv))
        self.assertTrue(re.search('\n10.000, 2, NPC2, 9.98, 284.9., -0.50, 1.56, 0.00, 0.00, 20.00', csv))
        self.assertTrue(re.search('\n10.000, 3, NPC3, 13.85, 296.9., -0.52, 1.56, 0.00, 0.00, 20.00', csv))
        self.assertTrue(re.search('\n10.000, 4, NPC4, 10.70, 329.9., -0.59, 1.55, 0.00, 0.00, 20.00', csv))
        self.assertTrue(re.search('\n10.000, 0, Ego, 10.20, 299.9., -0.53, 1.56, 0.00, 0.00, 20.00', csv))
        self.assertTrue(re.search('\n23.500, 1, NPC1, 19.45, 595.65, -0.83, 1.49, 6.28, 0.00, 30.00', csv))
        self.assertTrue(re.search('\n23.500, 2, NPC2, 18.35, 533.53, -0.85, 1.51, 6.28, 0.00, 8.72', csv))
        self.assertTrue(re.search('\n23.500, 3, NPC3, 23.67, 556.88, -0.84, 1.50, 6.28, 0.00, 17.00', csv))
        self.assertTrue(re.search('\n23.500, 4, NPC4, 21.61, 578.38, -0.83, 1.49, 6.28, 0.00, 8.72', csv))


if __name__ == "__main__":
    # execute only if run as a script

    unittest.main(verbosity=2)
