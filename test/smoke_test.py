import re
from sys import platform
from test_common import *
import unittest

ESMINI_PATH = '../'
COMMON_ARGS = '--headless --fixed_timestep 0.01 --record sim.dat '


class TestSuite(unittest.TestCase):

    def test_cut_in(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/cut-in.xosc'), COMMON_ARGS)
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading cut-in.xosc', log)  is not None)
        
        # Check some scenario events
        self.assertTrue(re.search('0.010.* CutInActStart == true, 0.0100 > 0.00 edge: none', log)  is not None)
        self.assertTrue(re.search('\n[789].* BrakeCondition == true, HWT: 0.70 > 0.70, edge rising', log)  is not None)
        self.assertTrue(re.search('\n21.[678].* ActStopCondition timer expired at 5.00 seconds', log)  is not None)

    def test_ltap_od(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/ltap-od.xosc'), COMMON_ARGS \
            + '--disable_controllers')
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading ltap-od.xosc', log)  is not None)
        self.assertTrue(re.search('.*Route::AddWaypoint Added intermediate waypoint 1 roadId 15 laneId -1', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('\n5.5.*Synchronize dist \(0.95\) < tolerance \(1.00\)', log)  is not None)
        self.assertTrue(re.search('\n9.5.* QuitCondition timer expired at 4.0. seconds', log)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('\n6.500.*, 0, Ego, 28.542, -7.876, 0.000, 1.779, 0.000, 0.000, 10.000', csv))
        self.assertTrue(re.search('\n6.500.*, 1, NPC, 24.456, 0.305, 0.000, 5.394, 0.000, 0.000, 7.000', csv))

    def test_trajectory(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/trajectory-test.xosc'), COMMON_ARGS \
            + '--disable_controllers')
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading trajectory-test.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('\n8.0.*FollowTrajectoryClothoidTrigger == true, element: FollowTrajectoryPLineEvent state: END_TRANSITION', log)  is not None)
        self.assertTrue(re.search('\n24.22.* FollowTrajectoryNurbsAction runningState -> endTransition -> completeState', log)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('\n4.100.*, 0, Ego, 115.042, 4.864, -3.006, 0.281, 0.032, 0.000, 16.000', csv))
        self.assertTrue(re.search('\n4.100.*, 1, Target, 129.91.*, 14.32.*, -3.46.*, 0.499.*', csv))
        self.assertTrue(re.search('\n11.100.*, 0, Ego, 200.713, 72.600, -2.443, 1.057, 6.263, 0.000, 16.000', csv))
        self.assertTrue(re.search('\n11.100.*, 1, Target, 206.050, 66.693, -2.491, 2.510, 6.281, 6.263', csv))
        self.assertTrue(re.search('\n17.250.*, 0, Ego, 217.345, 167.663, 1.989, 1.738, 6.209, 0.000, 16.000', csv))
        self.assertTrue(re.search('\n17.250.*, 1, Target, 210.675, 157.853, 1.321, 1.230, 6.216, 0.032, 14.873', csv))
        self.assertTrue(re.search('\n25.000.*, 0, Ego, 206.081, 288.506, 5.436, 1.188, 6.238, 0.000, 16.000', csv))
        self.assertTrue(re.search('\n25.000.*, 1, Target, 216.358, 307.857, 6.728, 0.965, 6.214', csv))

    def test_synchronize(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/synchronize.xosc'), COMMON_ARGS \
            + '--disable_controllers')
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading synchronize.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('\n9.9.* Synchronize dist \(0.92\) < tolerance \(1.00\)', log)  is not None)
        self.assertTrue(re.search('\n9.9.* Synchronize_NPC_Event complete after 1 execution', log)  is not None)
        self.assertTrue(re.search('\n19.75.* Free_Speed_Condition_NPC == true, distance 4.81 < tolerance \(5.00\), edge: rising', log)  is not None)
        self.assertTrue(re.search('\n19.75.* Triggering entity 0: Ego', log)  is not None)
        self.assertTrue(re.search('\n28.4.* All acts are done, quit now', log)  is not None)
        
        # Check vehicle key positions
        csv = generate_csv()

        self.assertTrue(re.search('\n10.000.*, 0, Ego, 10.20.*, 299.95.*, -0.52.*, 1.55.*, 0.00.*, 0.00.*, 20.00.*', csv))
        self.assertTrue(re.search('\n10.000.*, 1, NPC1, 6.70.*, 305.00.*, -0.53.*, 1.55.*, 0.00.*, 0.00.*, 20.00.*', csv))
        self.assertTrue(re.search('\n10.000.*, 2, NPC2, 9.98.*, 284.96.*, -0.49.*, 1.55.*, 0.00.*, 0.00.*, 20.00.*', csv))
        self.assertTrue(re.search('\n10.000.*, 3, NPC3, 13.85.*, 296.90.*, -0.52.*, 1.55.*, 0.00.*, 0.00.*, 20.00.*', csv))
        self.assertTrue(re.search('\n10.000.*, 4, NPC4, 10.70.*, 329.92.*, -0.58.*, 1.55.*, 0.00.*, 0.00.*, 20.00.*', csv))
        self.assertTrue(re.search('\n10.000.*, 5, MS1, 17.27.*, 299.76.*, -0.52.*, 1.55.*, 0.00.*, 0.00.*, 0.00.*', csv))
        self.assertTrue(re.search('\n10.000.*, 6, MS2, 23.37.*, 499.07.*, -0.84.*, 1.51.*, 0.00.*, 0.00.*, 0.00.*', csv))

        self.assertTrue(re.search('\n23.500.*, 1, NPC1, 19.44.*, 595.54.*, -0.82.*, 1.48.*, 6.28.*, 0.00.*, 30.00.*', csv))
        self.assertTrue(re.search('\n23.500.*, 2, NPC2, 18.35.*, 533.64.*, -0.84.*, 1.50.*, 6.28.*, 0.00.*, 8.75.*', csv))
        self.assertTrue(re.search('\n23.500.*, 3, NPC3, 23.67.*, 556.91.*, -0.84.*, 1.49.*, 6.28.*, 0.00.*, 17.00.*', csv))
        self.assertTrue(re.search('\n23.500.*, 4, NPC4, 21.61.*, 578.49.*, -0.83.*, 1.49.*, 6.28.*, 0.00.*, 8.75.*', csv))

    def test_left_hand_by_heading(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/left-hand-traffic_by_heading.xosc'), COMMON_ARGS \
            + '--disable_controllers')

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading left-hand-traffic_by_heading.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('\n3.76.* Lane change == true, rel_dist: 10.02 > 10.00, edge: rising', log)  is not None)
        self.assertTrue(re.search('\n5.76.* Lane change complete after 1 execution', log)  is not None)
        self.assertTrue(re.search('\n9.78.* QuitCondition timer expired at 4.01 seconds', log)  is not None)
        self.assertTrue(re.search('\n9.78.* All acts are done, quit now', log)  is not None)

        # Check vehicle key positions
        csv = generate_csv()

        self.assertTrue(re.search('\n2.500.*, 0, Ego, -7.54.*, 115.02.*, -0.17.*, 1.56.*, 0.002.*, 6.28.*, 30.00.*', csv))
        self.assertTrue(re.search('\n2.500.*, 1, OverTaker, -3.97.*, 115.01.*, -0.17.*, 1.56.*, 0.002.*, 0.00.*, 42.00.*', csv))
        self.assertTrue(re.search('\n4.380.*, 0, Ego, -7.19.*, 171.42.*, -0.29.*, 1.56.*, 0.002.*, 6.28.*, 30.00.*', csv))
        self.assertTrue(re.search('\n4.380.*, 1, OverTaker, -4.22.*, 193.97.*, -0.33.*, 1.61.*, 0.002.*, 0.00.*, 42.00.*', csv))
        self.assertTrue(re.search('\n9.000.*, 0, Ego, -5.64.*, 310.01.*, -0.54.*, 1.55.*, 0.002.*, 6.28.*, 30.00.*', csv))
        self.assertTrue(re.search('\n9.000.*, 1, OverTaker, -4.0(19|20).*, 388.02.*, -0.69.*, 1.54.*, 0.002.*, 0.00.*, 42.00.*', csv))

    def test_left_hand_using_road_rule(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/left-hand-traffic_using_road_rule.xosc'), COMMON_ARGS \
            + '--disable_controllers')

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading left-hand-traffic_using_road_rule.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('\n3.76.* Lane change == true, rel_dist: 10.02 > 10.00, edge: rising', log)  is not None)
        self.assertTrue(re.search('\n5.76.* Lane change complete after 1 execution', log)  is not None)
        self.assertTrue(re.search('\n9.78.* QuitCondition timer expired at 4.01 seconds', log)  is not None)
        self.assertTrue(re.search('\n9.78.* All acts are done, quit now', log)  is not None)
        
        # Check vehicle key positions
        csv = generate_csv()

        self.assertTrue(re.search('\n2.500.*, 0, Ego, -7.54.*, 115.02.*, -0.17.*, 1.56.*, 0.002.*, 0.00.*, 30.00.*', csv))
        self.assertTrue(re.search('\n2.500.*, 1, OverTaker, -3.97.*, 115.01.*, -0.17.*, 1.56.*, 0.002.*, 0.00.*, 42.00.*', csv))
        self.assertTrue(re.search('\n4.380.*, 0, Ego, -7.19.*, 171.42.*, -0.29.*, 1.56.*, 0.002.*, 0.00.*, 30.00.*', csv))
        self.assertTrue(re.search('\n4.380.*, 1, OverTaker, -4.22.*, 193.97.*, -0.33.*, 1.61.*, 0.002.*, 0.00.*, 42.00.*', csv))
        self.assertTrue(re.search('\n9.000.*, 0, Ego, -5.64.*, 310.01.*, -0.54.*, 1.55.*, 0.002.*, 0.00.*, 30.00.*', csv))
        self.assertTrue(re.search('\n9.000.*, 1, OverTaker, -4.0(19|20).*, 388.02.*, -0.69.*, 1.54.*, 0.002.*, 0.00.*, 42.00.*', csv))

    def test_routing(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/routing-test.xosc'), COMMON_ARGS)
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading routing-test.xosc', log)  is not None)
        self.assertTrue(re.search('.*Route::AddWaypoint Added waypoint 6: 261, 1, 50.00', log)  is not None)
        self.assertTrue(re.search('.*Route::AddWaypoint Added intermediate waypoint 7 roadId 260 laneId -1', log)  is not None)
        self.assertTrue(re.search('.*Route::AddWaypoint Added intermediate waypoint 11 roadId 220 laneId -1', log)  is not None)
        self.assertTrue(re.search('.*Route::AddWaypoint Added waypoint 12: 222, -1, 20.00', log)  is not None)
        self.assertTrue(re.search('\n25.47.* Route::AddWaypoint Added intermediate waypoint 3 roadId 280 laneId -1', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('\n25.47.* AquirePosition condition == true, distance 1.5. < tolerance \(2.00\), edge: rising', log)  is not None)
        self.assertTrue(re.search('\n25.47.: AquirePosition event complete after 1 execution', log)  is not None)
        self.assertTrue(re.search('\n38.84.* Stop condition == true, distance 1.6. < tolerance \(2.00\), edge: rising', log)  is not None)
        self.assertTrue(re.search('\n46.21.* QuitCondition timer expired at 4.0. seconds', log)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('\n7.00.*, 0, Ego, 291.875, 26.250, 0.000, 1.571, 0.00.*, 0.00.*, 50.00.*', csv))
        self.assertTrue(re.search('\n23.900, 0, Ego, 232.765, -3.753, 0.000, 6.063, 0.000, 0.000, 50.000.*', csv))
        self.assertTrue(re.search('\n42.170, 0, Ego, 623.968, -1.875, 0.000, 0.000, 0.000, 0.000, 0.050', csv))
        self.assertTrue(re.search('\n42.250, 0, Ego, 623.968, -1.875, 0.000, 0.000, 0.000, 0.000, 0.000', csv))

    def test_acc(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/acc-test.xosc'), COMMON_ARGS)
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading acc-test.xosc', log)  is not None)
        self.assertTrue(re.search('.*Ego New position:.*$\n^.*Pos\(20.00, -1.53, 0.00\) Rot\(0.00, 0.00, 0.00\) roadId 1 laneId -1 offset 0.00 t -1.53', log, re.MULTILINE))
        self.assertTrue(re.search('.*Target New position:.*$\n^.*Pos\(100.00, -1.53, 0.00\) Rot\(0.00, 0.00, 0.00\) roadId 1 laneId -1 offset 0.00 t -1.53', log, re.MULTILINE))

        # Check some scenario events
        self.assertTrue(re.search('^5.010: LaneChange1Condition == true, 5.0100 > 5.00 edge: rising', log, re.MULTILINE))
        self.assertTrue(re.search('^7.010: LaneChange2Condition == true, 7.0100 > 7.00 edge: rising', log, re.MULTILINE))
        self.assertTrue(re.search('^11.010: BrakeCondition == true, 11.0100 > 11.00 edge: rising', log, re.MULTILINE))
        self.assertTrue(re.search('^17.010: BrakeCondition == true, 17.0100 > 17.00 edge: rising', log, re.MULTILINE))
        self.assertTrue(re.search('^20.000: Brake2Condition == true, 20.0000 > 20.00 edge: rising', log, re.MULTILINE))
        self.assertTrue(re.search('^35.130: ActStopCondition == true, element: TargetBrake2Event state: END_TRANSITION, edge: rising', log, re.MULTILINE))

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('\n0.37.*, 0, Ego, 32.414, -1.535, 0.000, 0.000, 0.000, 0.000, 33.388', csv))
        self.assertTrue(re.search('\n0.37.*, 1, Target, 103.083, -1.535, 0.000, 0.000, 0.000, 0.000, 8.333', csv))
        self.assertTrue(re.search('\n4.89.*, 0, Ego, 122.579, -1.535, 0.000, 0.000, 0.000, 0.000, 10.180', csv))
        self.assertTrue(re.search('\n4.89.*, 1, Target, 140.750, -1.535, 0.000, 0.000, 0.000, 0.000, 8.333', csv))
        self.assertTrue(re.search('\n5.09.*, 0, Ego, 124.582, -1.535, 0.000, 0.000, 0.000, 0.000, 9.871', csv))
        self.assertTrue(re.search('\n5.09.*, 1, Target, 142.417, -1.514, 0.000, 0.060, 0.000, 0.000, 8.333.*', csv))
        self.assertTrue(re.search('\n7.71.*, 0, Ego, 153.805, -1.535, 0.000, 0.000, 0.000, 0.000, 9.360', csv))
        self.assertTrue(re.search('\n7.71.*, 1, Target, 164.25.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 8.33.*', csv))
        self.assertTrue(re.search('\n16.71.*, 0, Ego, 192.299, -1.535, 0.000, 0.000, 0.000, 0.000, 0.100', csv))
        self.assertTrue(re.search('\n16.71.*, 1, Target, 200.38.*, -1.53.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*', csv))
        self.assertTrue(re.search('\n25.11.*, 0, Ego, 289.549, -1.535, 0.000, 0.000, 0.000, 0.000, 11.312', csv))
        self.assertTrue(re.search('\n25.11.*, 1, Target, 309.41.*, -1.53.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 5.00.*', csv))
        self.assertTrue(re.search('\n34.11.*, 0, Ego, 341.474, -1.535, 0.000, 0.000, 0.000, 0.000, 5.001', csv))
        self.assertTrue(re.search('\n34.11.*, 1, Target, 354.415, -1.535, 0.000, 0.000, 0.000, 0.000, 5.000', csv))

    def test_conflicting_domains(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'EnvironmentSimulator/Unittest/xosc/conflicting-domains.xosc'), COMMON_ARGS)
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading conflicting-domains.xosc', log)  is not None)
        self.assertTrue(re.search('^0.00.*Ego New position:.*$\n^.*Pos\(20.00, -1.53, 0.00\) Rot\(0.00, 0.00, 0.00\) roadId 0 laneId -1 offset 0.00 t -1.53', log, re.MULTILINE))
        self.assertTrue(re.search('^0.00.*Init Ego TeleportAction standbyState -> startTransition -> runningState', log, re.MULTILINE))
        self.assertTrue(re.search('^0.00.*Init Ego LongitudinalAction standbyState -> startTransition -> runningState', log, re.MULTILINE))
        self.assertTrue(re.search('^0.00.*TeleportAction runningState -> endTransition -> completeState', log, re.MULTILINE))

        # Check some scenario events
        self.assertTrue(re.search('^2.00.*: Stopping Init Ego LongitudinalAction on conflicting longitudinal domain\(s\)', log, re.MULTILINE))
        self.assertTrue(re.search('^2.01.*: Speed action 2 standbyState -> startTransition -> runningState', log, re.MULTILINE))
        self.assertTrue(re.search('^2.01.*: Lane offset action 1 standbyState -> startTransition -> runningState', log, re.MULTILINE))

        self.assertTrue(re.search('^4.01.*: Stopping object Ego Lane offset action 1 on conflicting lateral domain\(s\)', log, re.MULTILINE))
        self.assertTrue(re.search('^4.02.*: Lane offset action 1 runningState -> endTransition -> completeState', log, re.MULTILINE))
        self.assertTrue(re.search('^4.02.*: Lane offset action 2 standbyState -> startTransition -> runningState', log, re.MULTILINE))

        self.assertTrue(re.search('^5.13.*: Lane offset action 2 runningState -> endTransition -> completeState', log, re.MULTILINE))
        self.assertTrue(re.search('^9.02.*: Speed action 2 runningState -> endTransition -> completeState', log, re.MULTILINE))
        self.assertTrue(re.search('^13.02.*: Stop condition timer expired at 4.01 seconds', log, re.MULTILINE))
        self.assertTrue(re.search('^13.02.*: Stop condition == true, speed: 20.00 >= 20.00, edge: none', log, re.MULTILINE))

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^4.00.*, 0, Ego, 35.88.*, -0.83.*, 0.00.*, 0.10.*, 0.00.*, 0.00.*, 7.97.*', csv, re.MULTILINE))
        self.assertTrue(re.search('^4.03.*, 0, Ego, 36.13.*, -0.82.*, 0.00.*, 6.27.*, 0.00.*, 0.00.*, 8.05.*', csv, re.MULTILINE))
        self.assertTrue(re.search('^5.13.*, 0, Ego, 46.74.*, -1.53.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 11.31.*', csv, re.MULTILINE))

    def test_swarm(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/swarm.xosc'), COMMON_ARGS + '--seed 5')
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading swarm.xosc', log)  is not None)
        self.assertTrue(re.search('Using specified seed 5', log)  is not None)
        self.assertTrue(re.search('^0.00.*Ego New position:.*$\n^.*Pos\(10.20, 299.87, -0.53\) Rot\(1.56, 0.00, 0.00\) roadId 0 laneId -3 offset 0.00 t -8.00', log, re.MULTILINE))
        self.assertTrue(re.search('^0.00.*Init Ego TeleportAction standbyState -> startTransition -> runningState', log, re.MULTILINE))
        self.assertTrue(re.search('^0.00.*Init Ego LongitudinalAction standbyState -> endTransition -> completeState', log, re.MULTILINE))

        # Check some scenario events
        self.assertTrue(re.search('^1.00.*: Swarm IR: 200.00, SMjA: 300.00, SMnA: 500.00, maxV: 75 vel: 30.00', log, re.MULTILINE))
        self.assertTrue(re.search('^60.01.*: SwarmStopTrigger == true, 60.0100 > 60.00 edge: none', log, re.MULTILINE))

        # Check vehicle key positions
        csv = generate_csv()

        self.assertTrue(re.search('^40.00.*, 0, Ego, 33.31.*, 699.01.*, -0.95.*, 1.45.*, 0.00.*, 0.00.*, 10.00.*', csv, re.MULTILINE))
        # Random generators differ on platforms => random traffic will be repeatable only per platform
        if platform == "win32":
            self.assertTrue(re.search('^1.59.*, 2, swarm2, 25.200, 576.852, -0.834, 1.491, 6.283, 0.000, 30.000', csv, re.MULTILINE))
            self.assertTrue(re.search('^40.00.*, 3, swarm3, 31.236, 680.144, -0.901, 1.463, 0.002, 0.000, 10.000', csv, re.MULTILINE))
            self.assertTrue(re.search('^40.00.*, 75, swarm75, 44.989, 765.411, -1.128, 1.444, 0.002, 0.000, 31.935', csv, re.MULTILINE))
            self.assertTrue(re.search('^40.00.*, 89, swarm89, 19.911, 690.914, -0.924, 4.603, 6.281, 0.000, 30.706', csv, re.MULTILINE))
        elif platform == "linux" or platform == "linux2":
            self.assertTrue(re.search('^1.59.*, 2, swarm2, 25.200, 576.852, -0.834, 1.491, 6.283, 0.000, 30.000', csv, re.MULTILINE))
            self.assertTrue(re.search('^40.00.*, 5, swarm5, 31.236, 680.144, -0.901, 1.463, 0.002, 0.000, 10.000', csv, re.MULTILINE))
            self.assertTrue(re.search('^40.00.*, 74, swarm74, 39.653, 721.787, -1.020, 1.454, 0.003, 0.000, 32.500', csv, re.MULTILINE))
            self.assertTrue(re.search('^40.00.*, 85, swarm85, 11.096, 640.678, -0.836, 4.615, 6.282, 0.000, 32.498', csv, re.MULTILINE))

    def test_conflicting_domains(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'EnvironmentSimulator/Unittest/xosc/conflicting-domains.xosc'), COMMON_ARGS)
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading conflicting-domains.xosc', log))
        self.assertTrue(re.search('^0.00.*Ego New position:.*$\n^.*Pos\(20.00, -1.53, 0.00\) Rot\(0.00, 0.00, 0.00\) roadId 0 laneId -1 offset 0.00 t -1.53', log, re.MULTILINE))
        self.assertTrue(re.search('^0.00.*Init Ego TeleportAction standbyState -> startTransition -> runningState', log, re.MULTILINE))
        self.assertTrue(re.search('^0.00.*Init Ego LongitudinalAction standbyState -> startTransition -> runningState', log, re.MULTILINE))
        self.assertTrue(re.search('^0.00.*TeleportAction runningState -> endTransition -> completeState', log, re.MULTILINE))

        # Check some scenario events
        self.assertTrue(re.search('^2.00.*: Stopping Init Ego LongitudinalAction on conflicting longitudinal domain\(s\)', log, re.MULTILINE))
        self.assertTrue(re.search('^2.01.*: Speed action 2 standbyState -> startTransition -> runningState', log, re.MULTILINE))
        self.assertTrue(re.search('^2.01.*: Lane offset action 1 standbyState -> startTransition -> runningState', log, re.MULTILINE))

        self.assertTrue(re.search('^4.01.*: Stopping object Ego Lane offset action 1 on conflicting lateral domain\(s\)', log, re.MULTILINE))
        self.assertTrue(re.search('^4.02.*: Lane offset action 1 runningState -> endTransition -> completeState', log, re.MULTILINE))
        self.assertTrue(re.search('^4.02.*: Lane offset action 2 standbyState -> startTransition -> runningState', log, re.MULTILINE))

        self.assertTrue(re.search('^5.13.*: Lane offset action 2 runningState -> endTransition -> completeState', log, re.MULTILINE))
        self.assertTrue(re.search('^9.02.*: Speed action 2 runningState -> endTransition -> completeState', log, re.MULTILINE))
        self.assertTrue(re.search('^13.02.*: Stop condition timer expired at 4.01 seconds', log, re.MULTILINE))
        self.assertTrue(re.search('^13.02.*: Stop condition == true, speed: 20.00 >= 20.00, edge: none', log, re.MULTILINE))

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^4.00.*, 0, Ego, 35.88.*, -0.83.*, 0.00.*, 0.10.*, 0.00.*, 0.00.*, 7.97.*', csv, re.MULTILINE))
        self.assertTrue(re.search('^4.03.*, 0, Ego, 36.13.*, -0.82.*, 0.00.*, 6.27.*, 0.00.*, 0.00.*, 8.05.*', csv, re.MULTILINE))
        self.assertTrue(re.search('^5.13.*, 0, Ego, 46.74.*, -1.53.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 11.31.*', csv, re.MULTILINE))

    def test_follow_ghost(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/follow_ghost.xosc'), COMMON_ARGS)
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading follow_ghost.xosc', log)  is not None)
        
        # Check some scenario events
        self.assertTrue(re.search('^0.030.* SpeedEvent1 complete after 1 execution', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^4.010.* LaneChangeCondition == true, 4.0100 > 4.00 edge: rising', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^20.040.* EventStopCondition == true, element: StopEvent state: COMPLETE, edge: none', log, re.MULTILINE)  is not None)
        
        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^-1.00.*, 1, Ego_ghost, 8.219, 62.098, -0.060, 1.567, 0.002.*', csv, re.MULTILINE))
        self.assertTrue(re.search('^5.50.*, 1, Ego_ghost, 8.606, 374.258, -0.672, 1.587, 0.002.*', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.50.*, 0, Ego, 10.056, 313.191, -0.553, 1.580, 0.002, 0.000.*', csv, re.MULTILINE))
        self.assertTrue(re.search('^9.20.*, 1, Ego_ghost, 20.109, 558.866, -0.841, 1.497, 6.283, 0.000.*', csv, re.MULTILINE))
        self.assertTrue(re.search('^9.50.*, 0, Ego, 11.408, 463.122, -0.811, 1.502, 0.001, 6.283.*', csv, re.MULTILINE))

    def test_heading_trig(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'EnvironmentSimulator/Unittest/xosc/traj-heading-trig.xosc'), COMMON_ARGS)
        
        # Check some initialization steps
        self.assertTrue(re.search('.*traj-heading-trig.xosc', log)  is not None)
        
        # Check some scenario events
        self.assertTrue(re.search('^0.020: MyLaneChangeEvent standbyState -> startTransition -> runningState', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^1.300: MyAlignOrientationStartCondition == true, distance 0.00 < tolerance \(1.00\), rel orientation \[0.05, 0.00, 0.00\] \(tolerance 0.05\), edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^12.700: MyActStopCondition == true, distance 0.00 < tolerance \(1.00\), abs orientation \[0.95, 0.00, 0.00\] \(tolerance 0.05\), edge: none', log, re.MULTILINE)  is not None)
        
        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^1.000, 0, Car0, 433.889, -1.247, 0.000, 0.040, 0.000, 0.000, 13.889', csv, re.MULTILINE))
        self.assertTrue(re.search('^12.690, 0, Car0, 582.264, 41.303, 0.000, 0.951, 0.000, 0.000, 13.889, 0.000, 0.917', csv, re.MULTILINE))

if __name__ == "__main__":
    # execute only if run as a script

    # Run next line instead to execute only one test
    # unittest.main(argv=['ignored', '-v', 'TestSuite.test_heading_trig'])
    
    unittest.main(verbosity=2)
