import re
from test_common import *
import unittest
import argparse

ESMINI_PATH = '../'
COMMON_ARGS = '--headless --fixed_timestep 0.01 --record sim.dat '
ALKS_PREFIX = './OSC-ALKS-scenarios/Scenarios/'


class TestSuite(unittest.TestCase):

    def test_ALKS_Scenario_4_1_1_FreeDriving(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.1_1_FreeDriving_TEMPLATE.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.1_1', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('0.00.* ActivateALKSControllerAct standbyState -> startTransition -> runningState', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n70.00.*, 0, Ego, 928.851, 434.773, 0.000, 0.737, 0.000, 0.000, 16.667', csv))

    def test_ALKS_Scenario_4_1_2_SwervingLeadVehicle(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.1_2_SwervingLeadVehicle_TEMPLATE.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.1_2', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('19.98.* SwerveAction2 initState -> startTransition -> runningState.*', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n22.170, 1, LeadVehicle, 412.833, -7.107, 0.000, 6.255, 0.000, 0.000, 16.667', csv))

    def test_ALKS_Scenario_4_1_3_SideVehicle(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.1_3_SideVehicle_TEMPLATE.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.1_3', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('0.00.* ActivateALKSControllerAct standbyState -> startTransition -> runningState', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n8.880.*, 1, SideVehicle, 153.00.*, -5.00.*, 0.00.*, 0.00.*,.*', csv))
        self.assertTrue(re.search('\n22.170.*, 1, SideVehicle, 374.50.*, -5.00.*, 0.00.*, 0.00.*,.*', csv))

    def test_ALKS_Scenario_4_2_1_FullyBlockingTarget(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.2_1_FullyBlockingTarget_TEMPLATE.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.2_1', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n22.000, 0, Ego, 371.667, -8.000, 0.000, 0.000, 0.000, 0.000, 16.667, 0.000, 4.610', csv))
        self.assertTrue(re.search('\n37.000, 0, Ego, 492.983, -8.000, 0.000, 0.000, 0.000, 0.000, 0.075, 0.000, 5.179', csv))

    def test_ALKS_Scenario_4_2_2_PartiallyBlockingTarget(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.2_2_PartiallyBlockingTarget_TEMPLATE.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.2_2', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('0.000.* Init TargetBlocking TeleportAction initState -> startTransition -> runningState', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n0.000, 1, TargetBlocking, 500.000, -9.500, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n22.700, 0, Ego, 383.333, -8.000, 0.000, 0.000, 0.000, 0.000, 16.667, 0.000, 0.245', csv))
        self.assertTrue(re.search('\n33.000, 0, Ego, 491.613, -8.000, 0.000, 0.000, 0.000, 0.000, 0.930, 0.000, 1.290', csv))

    def test_ALKS_Scenario_4_2_3_CrossingPedestrian(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.2_3_CrossingPedestrian_TEMPLATE.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.2_3', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('25.86.* CrossStartCondition == true, HWT: 3.(59|60) < 3.60, edge rising', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n28.000, 0, Ego, 471.667, -8.000, 0.000, 0.000, 0.000, 0.000, 16.667, 0.000, 1.298', csv))
        self.assertTrue(re.search('\n28.000, 1, TargetBlocking, 500.000, -10.028, 0.000, 1.570, 0.000, 0.000, 1.389, 0.000, 2.209', csv))
        self.assertTrue(re.search('\n30.720, 0, Ego, 491.238, -8.000, 0.000, 0.000, 0.000, 0.000, 1.407, 0.000, 0.231', csv))
        self.assertTrue(re.search('\n30.720, 1, TargetBlocking, 500.000, -6.250, 0.000, 1.570, 0.000, 0.000, 1.389, 0.000, 0.436', csv))
        self.assertTrue(re.search('\n32.500, 0, Ego, 504.769, -8.000, 0.000, 0.000, 0.000, 0.000, 13.867, 0.000, 1.547', csv))
        self.assertTrue(re.search('\n32.500, 1, TargetBlocking, 500.000, -3.778, 0.000, 1.570, 0.000, 0.000, 1.389, 0.000, 1.216', csv))

    def test_ALKS_Scenario_4_2_4_MultipleBlockingTargets(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.2_4_MultipleBlockingTargets_TEMPLATE.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.2_4', log)  is not None)

        # Check vehicle state
        csv = generate_csv()

        self.assertTrue(re.search('\n0.000, 0, Ego, 5.000, -8.000, 0.000, 0.000, 0.000, 0.000, 16.667, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n0.000, 1, TargetBlocking, 500.000, -8.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n22.500, 0, Ego, 380.000, -8.000, 0.000, 0.000, 0.000, 0.000, 16.667, 0.000, 3.287', csv))
        self.assertTrue(re.search('\n34.500, 0, Ego, 492.524, -8.000, 0.000, 0.000, 0.000, 0.000, 0.365, 0.000, 3.876', csv))

    def test_ALKS_Scenario_4_3_1_FollowLeadVehicleComfortable(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.3_1_FollowLeadVehicleComfortable_TEMPLATE.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.3_1', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('10.000.* VaryingSpeedStartCondition == true, 10.0000 >= 10.0000 edge: rising', log)  is not None)
        self.assertTrue(re.search('25.180: VaryingSpeedEvent2Start == true, element: VaryingSpeedAction state: END_TRANSITION, edge: rising', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n21.000, 0, Ego, 354.422, -8.000, 0.000, 0.000, 0.000, 0.000, 16.667, 0.000, 5.605', csv))
        self.assertTrue(re.search('\n21.000, 1, LeadVehicle, 430.253, -8.000, 0.000, 0.000, 0.000, 0.000, 21.846, 0.000, 6.127', csv))
        self.assertTrue(re.search('\n40.000, 0, Ego, 671.088, -8.000, 0.000, 0.000, 0.000, 0.000, 16.667, 0.000, 5.588', csv))
        self.assertTrue(re.search('\n40.000, 1, LeadVehicle, 746.231, -8.000, 0.000, 0.000, 0.000, 0.000, 11.667, 0.000, 4.140', csv))

    def test_ALKS_Scenario_4_3_2_FollowLeadVehicleEmergencyBrake(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.3_2_FollowLeadVehicleEmergencyBrake_TEMPLATE.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.3_2', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('10.000.* BrakeStartCondition == true, 10.0000 >= 10.0000 edge: rising', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n9.900, 0, Ego, 172.083, -8.000, 0.000, 0.000, 0.000, 0.000, 16.939', csv))
        self.assertTrue(re.search('\n9.900, 1, LeadVehicle, 208.333, -8.000, 0.000, 0.000, 0.000, 0.000, 16.667', csv))
        self.assertTrue(re.search('\n12.000, 0, Ego, 200.542, -8.000, 0.000, 0.000, 0.000, 0.000, 8.230', csv))
        self.assertTrue(re.search('\n12.000, 1, LeadVehicle, 224.075, -8.000, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n18.000, 0, Ego, 215.656, -8.000, 0.000, 0.000, 0.000, 0.000, 0.266', csv))
        self.assertTrue(re.search('\n18.000, 1, LeadVehicle, 224.075, -8.000, 0.000, 0.000, 0.000, 0.000, 0.000', csv))

    def test_ALKS_Scenario_4_4_1_CutInNoCollision(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.4_1_CutInNoCollision_TEMPLATE.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.4_1', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('9.100.* CutInStartCondition == true, rel_dist: 30.00 < 30.00, edge: rising', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n8.060.*, 0, Ego, 139.33.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n8.060.*, 1, CutInVehicle, 180.11.*, -11.50.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 11.1.*,.*', csv))
        self.assertTrue(re.search('\n13.000, 0, Ego, 211.134, -8.000, 0.000, 0.000, 0.000, 0.000, 10.964, 0.000, 4.455', csv))
        self.assertTrue(re.search('\n13.000, 1, CutInVehicle, 234.751, -8.000, 0.000, 0.000, 0.000, 0.000, 11.111, 0.000, 4.291', csv))

    def test_ALKS_Scenario_4_4_2_CutInUnavoidableCollision(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.4_2_CutInUnavoidableCollision_TEMPLATE.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.4_2', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('9.10.* CutInStartCondition == true, rel_dist: 10.00 < 10.00, edge: rising', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n8.00.*, 0, Ego, 138.33.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n8.00.*, 1, CutInVehicle, 159.44.*, -11.50.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 11.1.*,.*', csv))
        self.assertTrue(re.search('\n9.600, 0, Ego, 164.934, -8.000, 0.000, 0.000, 0.000, 0.000, 15.467, 0.000, 4.531', csv))
        self.assertTrue(re.search('\n9.600, 1, CutInVehicle, 177.180, -10.896, 0.000, 0.202, 0.000, 0.000, 11.111, 0.090, 3.169', csv))
        self.assertTrue(re.search('\n10.850, 0, Ego, 178.869, -8.000, 0.000, 0.000, 0.000, 0.000, 8.358, 0.000, 0.160', csv))
        self.assertTrue(re.search('\n10.850, 1, CutInVehicle, 190.735, -8.018, 0.000, 0.043, 0.000, 0.000, 11.111, -0.129, 5.152', csv))
        self.assertTrue(re.search('\n11.000, 0, Ego, 180.098, -8.000, 0.000, 0.000, 0.000, 0.000, 8.023, 0.000, 3.662', csv))
        self.assertTrue(re.search('\n11.000, 1, CutInVehicle, 192.401, -8.000, 0.000, 0.000, 0.000, 0.000, 11.111, -0.106, 3.631', csv))

    def test_ALKS_Scenario_4_5_1_CutOutFullyBlocking(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.5_1_CutOutFullyBlocking_TEMPLATE.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.5_1', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('24.17.* CutOutStartCondition == true, rel_dist: 49.93 < 50.00, edge: rising', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n24.500, 0, Ego, 417.718, -8.000, 0.000, 0.000, 0.000, 0.000, 16.747', csv))
        self.assertTrue(re.search('\n24.500, 1, TargetBlocking, 500.000, -8.000, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n30.000, 0, Ego, 485.333, -8.000, 0.000, 0.000, 0.000, 0.000, 4.410, 0.000, 2.296', csv))
        self.assertTrue(re.search('\n30.000, 1, TargetBlocking, 500.000, -8.000, 0.000, 0.000, 0.000, 0.000, 0.000', csv))

    def test_ALKS_Scenario_4_5_2_CutOutMultipleBlockingTargets(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.5_2_CutOutMultipleBlockingTargets_TEMPLATE.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.5_2', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('24.17.* CutOutStartCondition == true, rel_dist: 49.93 < 50.00, edge: rising', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n24.500, 0, Ego, 417.718, -8.000, 0.000, 0.000, 0.000, 0.000, 16.747', csv))
        self.assertTrue(re.search('\n24.500, 1, TargetBlocking, 500.000, -8.000, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n24.500, 2, TargetBlocking2, 515.000, -8.000, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n24.500, 3, LeadVehicle, 451.665, -7.877, 0.000, 0.043, 0.000, 0.000, 16.667, 0.024, 4.277', csv))
        self.assertTrue(re.search('\n32.000, 1, TargetBlocking, 500.000, -8.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n32.000, 2, TargetBlocking2, 515.000, -8.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n32.000, 3, LeadVehicle, 576.501, -4.500, 0.000, 0.000, 0.000, 0.000, 16.667, 0.000, 3.279', csv))

    def test_ALKS_Scenario_4_6_1_ForwardDetectionRange(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.6_1_ForwardDetectionRange_TEMPLATE.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.6_1', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n29.500.*, 0, Ego, 496.66.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n29.500.*, 1, TargetBlocking, 500.00.*, -13.25.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*,.*', csv))

    def test_ALKS_Scenario_4_6_2_LateralDetectionRange(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.6_2_LateralDetectionRange_TEMPLATE.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.6_2', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('10.00.* SwerveEventStart == true, 10.0000 >= 10.0000 edge: rising', log)  is not None)
        self.assertTrue(re.search('10.00.*: SwerveAction initState -> startTransition -> runningState', log)  is not None)
        self.assertTrue(re.search('26.10.*: SwerveEvent runningState -> endTransition -> completeState', log)  is not None)


        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n4.900.*, 0, Ego, 86.66.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n4.900.*, 1, SideVehicle, 86.66.*, -15.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n15.000.*, 0, Ego, 255.00.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n15.000, 1, SideVehicle, 255.000, -13.850, 0.000, 0.025, 0.000, 0.000, 16.667', csv))
        self.assertTrue(re.search('\n22.000.*, 0, Ego, 371.66.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n22.000, 1, SideVehicle, 371.667, -10.549, 0.000, 0.022, 0.000, 0.000, 16.667', csv))
        self.assertTrue(re.search('\n32.000.*, 0, Ego, 538.33.*, -8.00.*.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n32.000.*, 1, SideVehicle, 538.33.*, -9.75.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))

if __name__ == "__main__":
    # execute only if run as a script

    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--timeout", type=int, default=40, help="timeout per testcase")
    parser.add_argument("testcase", nargs="?", help="run only this testcase")
    args = parser.parse_args()

    print("timeout:", args.timeout, file=sys.stderr)
    set_timeout(args.timeout)

    if args.testcase:
        # Add test case name as argument to run only that test
        # example: smoke_test.py test_follow_ghost
        unittest.main(argv=['ignored', '-v', 'TestSuite.' + args.testcase])
    else:
        unittest.main(argv=[''], verbosity=2)
