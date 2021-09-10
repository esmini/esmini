import re
from test_common import *
import unittest

ESMINI_PATH = '../'
COMMON_ARGS = '--headless --fixed_timestep 0.01 --record sim.dat '
ALKS_PREFIX = './OSC-ALKS-scenarios/Scenarios/'


class TestSuite(unittest.TestCase):

    def test_ALKS_Scenario_4_1_1_FreeDriving(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.1_1_FreeDriving_TEMPLATE.xosc'), COMMON_ARGS)
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.1_1', log))
        
        # Check some scenario events
        self.assertTrue(re.search('0.00.* ActivateALKSControllerAct standbyState -> startTransition -> runningState', log))

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n70.00.*, 0, Ego, 928.57.*, 434.51.*, 0.00.*, 0.7.*, 0.00.*,.*', csv))

    def test_ALKS_Scenario_4_1_2_SwervingLeadVehicle(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.1_2_SwervingLeadVehicle_TEMPLATE.xosc'), COMMON_ARGS)
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.1_2', log))
        
        # Check some scenario events
        self.assertTrue(re.search('19.99.* SwerveAction2 standbyState -> startTransition -> runningState.*', log))

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n22.17., 1, LeadVehicle, 412.83.*, -7.11.*, 0.00.*, 6.25.*, .*', csv))
    
    def test_ALKS_Scenario_4_1_3_SideVehicle(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.1_3_SideVehicle_TEMPLATE.xosc'), COMMON_ARGS)
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.1_3', log))
        
        # Check some scenario events
        self.assertTrue(re.search('0.00.* ActivateALKSControllerAct standbyState -> startTransition -> runningState', log))

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n8.880.*, 1, SideVehicle, 153.00.*, -5.00.*, 0.00.*, 0.00.*,.*', csv))
        self.assertTrue(re.search('\n22.170.*, 1, SideVehicle, 374.50.*, -5.00.*, 0.00.*, 0.00.*,.*', csv))

    def test_ALKS_Scenario_4_2_1_FullyBlockingTarget(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.2_1_FullyBlockingTarget_TEMPLATE.xosc'), COMMON_ARGS)
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.2_1', log))

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n29.600.*, 0, Ego, 498.33.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6.*,.*', csv))

    def test_ALKS_Scenario_4_2_2_PartiallyBlockingTarget(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.2_2_PartiallyBlockingTarget_TEMPLATE.xosc'), COMMON_ARGS)
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.2_2', log))
        
        # Check some scenario events
        self.assertTrue(re.search('0.000.* Init TargetBlocking TeleportAction standbyState -> startTransition -> runningState', log))

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n0.000.*, 1, TargetBlocking, 500.00.*, -9.50.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*', csv))
        self.assertTrue(re.search('\n29.620.*, 0, Ego, 498.6.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n29.620.*, 1, TargetBlocking, 500.00.*, -9.50.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00', csv))

    def test_ALKS_Scenario_4_2_3_CrossingPedestrian(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.2_3_CrossingPedestrian_TEMPLATE.xosc'), COMMON_ARGS)
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.2_3', log))
        
        # Check some scenario events
        self.assertTrue(re.search('25.87.* CrossStartCondition == true, HWT: 3.(59|60) < 3.60, edge rising', log))

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n29.710.*, 0, Ego, 500.16.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.66.*,.*', csv))
        self.assertTrue(re.search('\n29.710.*, 1, TargetBlocking, 500.00.*, -7.65.*, 0.00.*, 1.57.*, 0.00.*, 0.00.*, 1.38.*,.*', csv))

    def test_ALKS_Scenario_4_2_4_MultipleBlockingTargets(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.2_4_MultipleBlockingTargets_TEMPLATE.xosc'), COMMON_ARGS)
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.2_4', log))
        
        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n0.000.*, 1, TargetBlocking, 500.00.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*', csv))
        self.assertTrue(re.search('\n0.000.*, 2, TargetBlocking2, 515.00.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*', csv))
        self.assertTrue(re.search('\n29.630.*, 0, Ego, 498.83.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.66.*,.*', csv))

    def test_ALKS_Scenario_4_3_1_FollowLeadVehicleComfortable(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.3_1_FollowLeadVehicleComfortable_TEMPLATE.xosc'), COMMON_ARGS)
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.3_1', log))
        
        # Check some scenario events
        self.assertTrue(re.search('10.010.* VaryingSpeedStartCondition == true, 10.0100 >= 10.00 edge: rising', log))
        self.assertTrue(re.search('25.020.* VaryingSpeedEvent2Start == true, element: VaryingSpeedAction state: END_TRANSITION, edge: rising', log))

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n21.000.*, 0, Ego, 355.00.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n21.000.*, 1, LeadVehicle, 429.14.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 21.66.*,.*', csv))
        self.assertTrue(re.search('\n33.030.*, 0, Ego, 555.5.*., -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n33.030.*, 1, LeadVehicle, 657.67.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 13.65.*,.*', csv))

    def test_ALKS_Scenario_4_3_2_FollowLeadVehicleEmergencyBrake(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.3_2_FollowLeadVehicleEmergencyBrake_TEMPLATE.xosc'), COMMON_ARGS)
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.3_2', log))
        
        # Check some scenario events
        self.assertTrue(re.search('10.010.* BrakeStartCondition == true, 10.0100 >= 10.00 edge: rising', log))

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n9.900.*, 0, Ego, 170.00.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n9.900.*, 1, LeadVehicle, 208.33.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n11.000.*, 0, Ego, 188.33.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n11.000.*, 1, LeadVehicle, 221.81.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*,.*', csv))
        self.assertTrue(re.search('\n13.000.*, 0, Ego, 221.66.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n13.000.*, 1, LeadVehicle, 224.24.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*,.*', csv))

    def test_ALKS_Scenario_4_4_1_CutInNoCollision(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.4_1_CutInNoCollision_TEMPLATE.xosc'), COMMON_ARGS)
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.4_1', log))
        
        # Check some scenario events
        self.assertTrue(re.search('9.110.* CutInStartCondition == true, rel_dist: 30.00 < 30.00, edge: rising', log))

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n8.060.*, 0, Ego, 139.33.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n8.060.*, 1, CutInVehicle, 180.11.*, -11.50.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 11.1.*,.*', csv))
        self.assertTrue(re.search('\n13.000.*, 0, Ego, 221.66.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n13.000.*, 1, CutInVehicle, 235.00.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 11.1.*,.*', csv))

    def test_ALKS_Scenario_4_4_2_CutInUnavoidableCollision(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.4_2_CutInUnavoidableCollision_TEMPLATE.xosc'), COMMON_ARGS)
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.4_2', log))
        
        # Check some scenario events
        self.assertTrue(re.search('9.11.* CutInStartCondition == true, rel_dist: 10.00 < 10.00, edge: rising', log))

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n8.00.*, 0, Ego, 138.33.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n8.00.*, 1, CutInVehicle, 159.44.*, -11.50.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 11.1.*,.*', csv))
        self.assertTrue(re.search('\n9.60.*, 0, Ego, 165.00.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n9.60.*, 1, CutInVehicle, 177.22.*, -10.91.*, 0.00.*, 0.19.*, 0.00.*, 0.00.*, 11.11.*,.*', csv))
        self.assertTrue(re.search('\n10.85.*, 0, Ego, 185.83.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n10.85.*, 1, CutInVehicle, 191.11.*, -8.02.*, 0.00.*, 0.04.*, 0.00.*, 0.00.*, 11.11.*,.*', csv))
        self.assertTrue(re.search('\n11.00.*, 0, Ego, 188.33.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n11.00.*, 1, CutInVehicle, 192.77.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 11.11.*,.*', csv))

    def test_ALKS_Scenario_4_5_1_CutOutFullyBlocking(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.5_1_CutOutFullyBlocking_TEMPLATE.xosc'), COMMON_ARGS)
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.5_1', log))
        
        # Check some scenario events
        self.assertTrue(re.search('24.18.* CutOutStartCondition == true, rel_dist: 49.93 < 50.00, edge: rising', log))

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n24.500.*, 0, Ego, 413.33.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n24.500.*, 1, TargetBlocking, 500.00.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*,.*', csv))
        self.assertTrue(re.search('\n28.000.*, 0, Ego, 471.66.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n28.000.*, 1, TargetBlocking, 500.00.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*,.*', csv))

    def test_ALKS_Scenario_4_5_2_CutOutMultipleBlockingTargets(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.5_2_CutOutMultipleBlockingTargets_TEMPLATE.xosc'), COMMON_ARGS)
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.5_2', log))
        
        # Check some scenario events
        self.assertTrue(re.search('24.18.* CutOutStartCondition == true, rel_dist: 49.93 < 50.00, edge: rising', log))

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n24.500.*, 0, Ego, 413.33.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n24.500.*, 1, TargetBlocking, 500.00.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*,.*', csv))
        self.assertTrue(re.search('\n24.500.*, 2, TargetBlocking2, 515.00.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*,.*', csv))
        self.assertTrue(re.search('\n24.500.*, 3, LeadVehicle, 451.66.*, -7.88.*, 0.00.*, 0.04.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n28.000.*, 0, Ego, 471.66.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n28.000.*, 1, TargetBlocking, 500.00.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*,.*', csv))

    def test_ALKS_Scenario_4_6_1_ForwardDetectionRange(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.6_1_ForwardDetectionRange_TEMPLATE.xosc'), COMMON_ARGS)
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.6_1', log))
        
        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n29.500.*, 0, Ego, 496.66.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n29.500.*, 1, TargetBlocking, 500.00.*, -13.25.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*,.*', csv))

    def test_ALKS_Scenario_4_6_2_LateralDetectionRange(self):
        log = run_scenario(os.path.join(ALKS_PREFIX + 'ALKS_Scenario_4.6_2_LateralDetectionRange_TEMPLATE.xosc'), COMMON_ARGS)
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*ALKS_Scenario_4.6_2', log))
        
        # Check some scenario events
        self.assertTrue(re.search('10.01.* SwerveEventStart == true, 10.0100 >= 10.00 edge: rising', log))
        self.assertTrue(re.search('10.02.*: SwerveAction standbyState -> startTransition -> runningState', log))
        self.assertTrue(re.search('24.93.*: SwerveEvent runningState -> endTransition -> completeState', log))


        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n4.900.*, 0, Ego, 86.66.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n4.900.*, 1, SideVehicle, 86.66.*, -15.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n15.000.*, 0, Ego, 255.00.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n15.000.*, 1, SideVehicle, 255.00.*, -13.67.*, 0.00.*, 0.02.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n22.000.*, 0, Ego, 371.66.*, -8.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n22.000.*, 1, SideVehicle, 371.66.*, -10.22.*, 0.00.*, 0.019.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n32.000.*, 0, Ego, 538.33.*, -8.00.*.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))
        self.assertTrue(re.search('\n32.000.*, 1, SideVehicle, 538.33.*, -9.75.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 16.6[67].*,.*', csv))

if __name__ == "__main__":
    # execute only if run as a script

    unittest.main(verbosity=2)
