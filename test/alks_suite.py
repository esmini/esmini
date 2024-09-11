import re
from test_common import *
import unittest
import argparse

# run all scenarios from https://github.com/asam-oss/OSC-ALKS-scenarios
# check some key events and positions

ESMINI_PATH = '../'
COMMON_ARGS = '--headless --fixed_timestep 0.05 --record sim.dat '
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
        self.assertTrue(re.search('\n70.000, 0, Ego, 928.843, 434.766, 0.000, 0.737, 0.000, 0.000, 16.667, -0.013, 3.245', csv))

    def test_ALKS_Scenario_4_1_2_SwervingLeadVehicle(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.1_2_SwervingLeadVehicle_TEMPLATE.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.1_2', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('20.050: SwerveAction2 initState -> startTransition -> runningState.*', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n22.150, 1, LeadVehicle, 412.500, -7.047, 0.000, 6.256, 0.000, 0.000, 16.667, -0.001, 5.470', csv))

    def test_ALKS_Scenario_4_1_3_SideVehicle(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.1_3_SideVehicle_TEMPLATE.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.1_3', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('0.00.* ActivateALKSControllerAct standbyState -> startTransition -> runningState', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n8.850, 0, Ego, 152.500, -8.000, 0.000, 0.000, 0.000, 0.000, 16.667, 0.000, 0.455', csv))
        self.assertTrue(re.search('\n8.850, 1, SideVehicle, 152.500, -5.000, 0.000, 0.000, 0.000, 0.000, 16.667, 0.000, 0.455', csv))
        self.assertTrue(re.search('\n58.000, 0, Ego, 832.535, 261.961, 0.000, 1.200, 0.000, 0.000, 16.667, 0.000, 3.586', csv))
        self.assertTrue(re.search('\n58.000, 1, SideVehicle, 831.043, 266.404, 0.000, 1.200, 0.000, 0.000, 16.667, 0.000, 3.586', csv))
        self.assertTrue(re.search('\n230.000, 0, Ego, 3400.939, 1182.297, 0.000, 0.000, 0.000, 0.000, 16.667, 0.000, 0.789', csv))
        self.assertTrue(re.search('\n230.000, 1, SideVehicle, 3400.939, 1185.297, 0.000, 0.000, 0.000, 0.000, 16.667, 0.000, 0.789', csv))

    def test_ALKS_Scenario_4_2_1_FullyBlockingTarget(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.2_1_FullyBlockingTarget_TEMPLATE.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.2_1', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n22.000, 0, Ego, 371.667, -8.000, 0.000, 0.000, 0.000, 0.000, 16.667, 0.000, 4.610', csv))
        self.assertTrue(re.search('\n37.000, 0, Ego, 492.973, -8.000, 0.000, 0.000, 0.000, 0.000, 0.079, 0.000, 3.256', csv))

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
        self.assertTrue(re.search('\n35.000, 0, Ego, 492.654, -8.000, 0.000, 0.000, 0.000, 0.000, 0.275, 0.000, 2.372', csv))

    def test_ALKS_Scenario_4_2_3_CrossingPedestrian(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.2_3_CrossingPedestrian_TEMPLATE.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.2_3', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('25.900: CrossStartCondition == true, HWT: 3.55 < 3.60, edge rising', log)  is not None)

        # Check vehicle state
        csv = generate_csv()

        self.assertTrue(re.search('\n28.000, 0, Ego, 471.667, -8.000, 0.000, 0.000, 0.000, 0.000, 16.667, 0.000, 1.298', csv))
        self.assertTrue(re.search('\n28.000, 1, TargetBlocking, 500.000, -10.083, 0.000, 1.570, 0.000, 0.000, 1.389, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n31.000, 0, Ego, 491.850, -8.000, 0.000, 0.000, 0.000, 0.000, 3.021, 0.000, 0.468', csv))
        self.assertTrue(re.search('\n31.000, 1, TargetBlocking, 500.000, -5.917, 0.000, 1.570, 0.000, 0.000, 1.389, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n32.500, 0, Ego, 503.994, -8.000, 0.000, 0.000, 0.000, 0.000, 13.521, 0.000, 5.249', csv))
        self.assertTrue(re.search('\n32.500, 1, TargetBlocking, 500.000, -3.833, 0.000, 1.570, 0.000, 0.000, 1.389, 0.000, 0.000', csv))

    def test_ALKS_Scenario_4_2_4_MultipleBlockingTargets(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.2_4_MultipleBlockingTargets_TEMPLATE.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.2_4', log)  is not None)

        # Check vehicle state
        csv = generate_csv()

        self.assertTrue(re.search('\n0.000, 0, Ego, 5.000, -8.000, 0.000, 0.000, 0.000, 0.000, 16.667, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n0.000, 1, TargetBlocking, 500.000, -8.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n22.500, 0, Ego, 380.000, -8.000, 0.000, 0.000, 0.000, 0.000, 16.667, 0.000, 3.287', csv))
        self.assertTrue(re.search('\n34.500, 0, Ego, 492.490, -8.000, 0.000, 0.000, 0.000, 0.000, 0.376, 0.000, 1.918', csv))

    def test_ALKS_Scenario_4_3_1_FollowLeadVehicleComfortable(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.3_1_FollowLeadVehicleComfortable_TEMPLATE.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.3_1', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('10.000.* VaryingSpeedStartCondition == true, 10.0000 >= 10.0000 edge: rising', log)  is not None)
        self.assertTrue(re.search('25.000.* VaryingSpeedEvent2Start == true, element: VaryingSpeedAction state: END_TRANSITION, edge: rising', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n21.000, 0, Ego, 354.056, -8.000, 0.000, 0.000, 0.000, 0.000, 16.667, 0.000, 4.559', csv))
        self.assertTrue(re.search('\n21.000, 1, LeadVehicle, 429.292, -8.000, 0.000, 0.000, 0.000, 0.000, 21.667, 0.000, 3.379', csv))
        self.assertTrue(re.search('\n54.400, 0, Ego, 881.415, -8.000, 0.000, 0.000, 0.000, 0.000, 12.161, 0.000, 2.693', csv))
        self.assertTrue(re.search('\n54.400, 1, LeadVehicle, 908.708, -8.000, 0.000, 0.000, 0.000, 0.000, 11.667, 0.000, 3.406', csv))

    def test_ALKS_Scenario_4_3_2_FollowLeadVehicleEmergencyBrake(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.3_2_FollowLeadVehicleEmergencyBrake_TEMPLATE.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.3_2', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('10.000.* BrakeStartCondition == true, 10.0000 >= 10.0000 edge: rising', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n9.900, 0, Ego, 170.000, -8.000, 0.000, 0.000, 0.000, 0.000, 16.667, 0.000, 0.190', csv))
        self.assertTrue(re.search('\n9.900, 1, LeadVehicle, 208.333, -8.000, 0.000, 0.000, 0.000, 0.000, 16.667, 0.000, 0.190', csv))
        self.assertTrue(re.search('\n12.000, 0, Ego, 198.767, -8.000, 0.000, 0.000, 0.000, 0.000, 8.580, 0.000, 5.826', csv))
        self.assertTrue(re.search('\n12.000, 1, LeadVehicle, 223.741, -8.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.231', csv))
        self.assertTrue(re.search('\n18.800, 0, Ego, 215.443, -8.000, 0.000, 0.000, 0.000, 0.000, 0.185, 0.000, 2.008', csv))
        self.assertTrue(re.search('\n18.800, 1, LeadVehicle, 223.741, -8.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.231', csv))

    def test_ALKS_Scenario_4_4_1_CutInNoCollision(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.4_1_CutInNoCollision_TEMPLATE.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.4_1', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('9.100.* CutInStartCondition == true, rel_dist: 30.00 < 30.00, edge: rising', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n8.050, 0, Ego, 139.167, -8.000, 0.000, 0.000, 0.000, 0.000, 16.667, 0.000, 0.059', csv))
        self.assertTrue(re.search('\n8.050, 1, CutInVehicle, 180.000, -11.500, 0.000, 0.000, 0.000, 0.000, 11.111, 0.000, 4.228', csv))
        self.assertTrue(re.search('\n10.400, 0, Ego, 178.333, -8.000, 0.000, 0.000, 0.000, 0.000, 16.667, 0.000, 5.150', csv))
        self.assertTrue(re.search('\n10.400, 1, CutInVehicle, 206.005, -9.899, 0.000, 0.179, 0.000, 0.000, 11.111, 0.010, 3.433', csv))
        self.assertTrue(re.search('\n13.500, 0, Ego, 216.513, -8.000, 0.000, 0.000, 0.000, 0.000, 10.871, 0.000, 0.309', csv))
        self.assertTrue(re.search('\n13.500, 1, CutInVehicle, 240.307, -8.000, 0.000, 0.000, 0.000, 0.000, 11.111, 0.000, 1.315', csv))

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
        self.assertTrue(re.search('\n9.600, 0, Ego, 164.975, -8.000, 0.000, 0.000, 0.000, 0.000, 15.667, 0.000, 4.539', csv))
        self.assertTrue(re.search('\n9.600, 1, CutInVehicle, 177.185, -10.896, 0.000, 0.189, 0.000, 0.000, 11.111, 0.099, 3.169', csv))
        self.assertTrue(re.search('\n10.050, 0, Ego, 171.188, -8.000, 0.000, 0.000, 0.000, 0.000, 11.796, 0.000, 2.889', csv))
        self.assertTrue(re.search('\n10.050, 1, CutInVehicle, 182.030, -9.649, 0.000, 0.273, 0.000, 0.000, 11.111, 0.010, 4.888', csv))
        self.assertTrue(re.search('\n11.000, 0, Ego, 180.321, -8.000, 0.000, 0.000, 0.000, 0.000, 7.947, 0.000, 3.301', csv))
        self.assertTrue(re.search('\n11.000, 1, CutInVehicle, 192.401, -8.000, 0.000, 0.000, 0.000, 0.000, 11.111, -0.129, 3.631', csv))

    def test_ALKS_Scenario_4_5_1_CutOutFullyBlocking(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.5_1_CutOutFullyBlocking_TEMPLATE.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.5_1', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('24.200: CutOutStartCondition == true, rel_dist: 49.43 < 50.00, edge: rising', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n24.500, 0, Ego, 413.333, -8.000, 0.000, 0.000, 0.000, 0.000, 16.667, 0.000, 4.277', csv))
        self.assertTrue(re.search('\n24.500, 1, TargetBlocking, 500.000, -8.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n24.500, 2, LeadVehicle, 451.666, -7.898, 0.000, 0.034, 0.000, 0.000, 16.667, 0.025, 4.277', csv))
        self.assertTrue(re.search('\n30.000, 0, Ego, 483.918, -8.000, 0.000, 0.000, 0.000, 0.000, 4.994, 0.000, 3.219', csv))
        self.assertTrue(re.search('\n30.000, 1, TargetBlocking, 500.000, -8.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n30.000, 2, LeadVehicle, 543.168, -4.500, 0.000, 0.000, 0.000, 0.000, 16.667, 0.000, 2.288', csv))

    def test_ALKS_Scenario_4_5_2_CutOutMultipleBlockingTargets(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.5_2_CutOutMultipleBlockingTargets_TEMPLATE.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.5_2', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('24.200: CutOutStartCondition == true, rel_dist: 49.43 < 50.00, edge: rising', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n24.500, 0, Ego, 413.333, -8.000, 0.000, 0.000, 0.000, 0.000, 16.667, 0.000, 4.277', csv))
        self.assertTrue(re.search('\n24.500, 1, TargetBlocking, 500.000, -8.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n24.500, 2, TargetBlocking2, 515.000, -8.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n24.500, 3, LeadVehicle, 451.666, -7.898, 0.000, 0.034, 0.000, 0.000, 16.667, 0.025, 4.277', csv))
        self.assertTrue(re.search('\n32.000, 0, Ego, 490.237, -8.000, 0.000, 0.000, 0.000, 0.000, 1.710, 0.000, 1.955', csv))
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
        self.assertTrue(re.search('10.00.* SwerveAction initState -> startTransition -> runningState', log)  is not None)
        self.assertTrue(re.search('26.10.* SwerveEvent runningState -> endTransition -> completeState', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n4.900.*, 0, Ego, 86.66.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n4.900.*, 1, SideVehicle, 86.66.*, -15.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n15.000, 0, Ego, 255.000, -8.000, 0.000, 0.000, 0.000, 0.000, 16.667, 0.000, 4.286', csv))
        self.assertTrue(re.search('\n15.000, 1, SideVehicle, 255.000, -13.867, 0.000, 0.025, 0.000, 0.000, 16.667, 0.001, 4.286', csv))
        self.assertTrue(re.search('\n22.000, 0, Ego, 371.667, -8.000, 0.000, 0.000, 0.000, 0.000, 16.667, 0.000, 4.610', csv))
        self.assertTrue(re.search('\n22.000, 1, SideVehicle, 371.667, -10.564, 0.000, 0.022, 0.000, 0.000, 16.667, -0.001, 4.610', csv))
        self.assertTrue(re.search('\n32.000, 0, Ego, 538.333, -8.000, 0.000, 0.000, 0.000, 0.000, 16.667, 0.000, 3.279', csv))
        self.assertTrue(re.search('\n32.000, 1, SideVehicle, 538.333, -9.750, 0.000, 0.000, 0.000, 0.000, 16.667, 0.000, 3.279', csv))

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
