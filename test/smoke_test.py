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
        self.assertTrue(re.search('\n8.00.*FollowTrajectoryClothoidTrigger == true, element: FollowTrajectoryPLineEvent state: END_TRANSITION', log)  is not None)
        self.assertTrue(re.search('\n24.22.* FollowTrajectoryNurbsAction runningState -> endTransition -> completeState', log)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('\n4.100.*, 0, Ego, 115.042, 4.864, -3.006, 0.281, 0.032, 0.000, 16.000', csv))
        self.assertTrue(re.search('\n4.100, 1, Target, 129.852, 14.341, -3.467, 0.506, 0.020, 0.002, 24.524', csv))
        self.assertTrue(re.search('\n11.100.*, 0, Ego, 200.713, 72.600, -2.443, 1.057, 6.263, 0.000, 16.000', csv))
        self.assertTrue(re.search('\n11.100, 1, Target, 205.945, 66.378, -2.497, 2.507, 6.281, 6.263, 17.500', csv))
        self.assertTrue(re.search('\n17.250.*, 0, Ego, 217.345, 167.663, 1.989, 1.738, 6.209, 0.000, 16.000', csv))
        self.assertTrue(re.search('\n17.250, 1, Target, 210.661, 157.753, 1.314, 1.228, 6.216, 0.032, 14.839', csv))
        self.assertTrue(re.search('\n25.000.*, 0, Ego, 206.081, 288.506, 5.436, 1.188, 6.238, 0.000, 16.000', csv))
        self.assertTrue(re.search('\n25.000, 1, Target, 216.140, 307.534, 6.701, 0.969, 6.214, 0.000, 21.125', csv))

    def test_synchronize(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/synchronize.xosc'), COMMON_ARGS \
            + '--disable_controllers')
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading synchronize.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('\n9.9.* Synchronize dist \(0.92\) < tolerance \(1.00\)', log)  is not None)
        self.assertTrue(re.search('\n9.9.* Synchronize_NPC_Event complete after 1 execution', log)  is not None)
        self.assertTrue(re.search('\n19.74.* Free_Speed_Condition_NPC == true, distance 4.81 < tolerance \(5.00\), edge: rising', log)  is not None)
        self.assertTrue(re.search('\n19.74.* Triggering entity 0: Ego', log)  is not None)
        self.assertTrue(re.search('\n32.180: All acts are done, quit now', log)  is not None)
        
        # Check vehicle key positions
        csv = generate_csv()

        self.assertTrue(re.search('\n10.000.*, 0, Ego, 10.20.*, 299.95.*, -0.52.*, 1.55.*, 0.00.*, 0.00.*, 20.00.*', csv))
        self.assertTrue(re.search('\n10.000.*, 1, NPC1, 6.70.*, 305.00.*, -0.53.*, 1.55.*, 0.00.*, 0.00.*, 20.00.*', csv))
        self.assertTrue(re.search('\n10.000.*, 2, NPC2, 9.98.*, 284.96.*, -0.49.*, 1.55.*, 0.00.*, 0.00.*, 20.00.*', csv))
        self.assertTrue(re.search('\n10.000.*, 3, NPC3, 13.85.*, 296.90.*, -0.52.*, 1.55.*, 0.00.*, 0.00.*, 20.00.*', csv))
        self.assertTrue(re.search('\n10.000.*, 4, NPC4, 10.70.*, 329.92.*, -0.58.*, 1.55.*, 0.00.*, 0.00.*, 20.00.*', csv))
        self.assertTrue(re.search('\n10.000.*, 5, MS1, 17.27.*, 299.76.*, -0.52.*, 1.55.*, 0.00.*, 0.00.*, 0.00.*', csv))
        self.assertTrue(re.search('\n10.000.*, 6, MS2, 23.37.*, 499.07.*, -0.84.*, 1.51.*, 0.00.*, 0.00.*, 0.00.*', csv))

        self.assertTrue(re.search('\n25.000, 1, NPC1, 22.644, 630.990, -0.831, 1.476, 0.001, 0.000, 30.000', csv))
        self.assertTrue(re.search('\n25.000, 2, NPC2, 20.598, 565.401, -0.839, 1.495, 6.283, 0.000, 9.928', csv))
        self.assertTrue(re.search('\n25.000, 3, NPC3, 25.708, 583.161, -0.832, 1.489, 6.283, 0.000, 17.000', csv))
        self.assertTrue(re.search('\n25.000, 4, NPC4, 24.312, 610.213, -0.826, 1.481, 6.283, 0.000, 9.928', csv))

    def test_left_hand_by_heading(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/left-hand-traffic_by_heading.xosc'), COMMON_ARGS \
            + '--disable_controllers')

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading left-hand-traffic_by_heading.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('\n3.75.* Lane change == true, rel_dist: 10.02 > 10.00, edge: rising', log)  is not None)
        self.assertTrue(re.search('\n5.75.* Lane change complete after 1 execution', log)  is not None)
        self.assertTrue(re.search('\n9.76.* QuitCondition timer expired at 4.00 seconds', log)  is not None)
        self.assertTrue(re.search('\n9.77.* All acts are done, quit now', log)  is not None)

        # Check vehicle key positions
        csv = generate_csv()

        self.assertTrue(re.search('\n2.500.*, 0, Ego, -7.54.*, 115.02.*, -0.17.*, 1.56.*, 0.002.*, 6.28.*, 30.00.*', csv))
        self.assertTrue(re.search('\n2.500.*, 1, OverTaker, -3.97.*, 115.01.*, -0.17.*, 1.56.*, 0.002.*, 0.00.*, 42.00.*', csv))
        self.assertTrue(re.search('\n4.380.*, 0, Ego, -7.19.*, 171.42.*, -0.29.*, 1.56.*, 0.002.*, 6.28.*, 30.00.*', csv))
        self.assertTrue(re.search('\n4.380.*, 1, OverTaker, -4.22.*, 193.97.*, -0.33.*, 1.61.*, 0.002.*, 0.00.*, 42.00.*', csv))
        self.assertTrue(re.search('\n9.000.*, 0, Ego, -5.64.*, 310.01.*, -0.54.*, 1.55.*, 0.002.*, 6.28.*, 30.00.*', csv))
        self.assertTrue(re.search('\n9.000, 1, OverTaker, -4.019, 387.992, -0.697, 1.544, 0.002, 0.000, 42.000, 0.000, 5.575', csv))

    def test_left_hand_using_road_rule(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/left-hand-traffic_using_road_rule.xosc'), COMMON_ARGS \
            + '--disable_controllers')

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading left-hand-traffic_using_road_rule.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('\n3.75.* Lane change == true, rel_dist: 10.02 > 10.00, edge: rising', log)  is not None)
        self.assertTrue(re.search('\n5.75.* Lane change complete after 1 execution', log)  is not None)
        self.assertTrue(re.search('\n9.76.* QuitCondition timer expired at 4.00 seconds', log)  is not None)
        self.assertTrue(re.search('\n9.77.* All acts are done, quit now', log)  is not None)
        
        # Check vehicle key positions
        csv = generate_csv()

        self.assertTrue(re.search('\n2.500.*, 0, Ego, -7.54.*, 115.02.*, -0.17.*, 1.56.*, 0.002.*, 0.00.*, 30.00.*', csv))
        self.assertTrue(re.search('\n2.500.*, 1, OverTaker, -3.97.*, 115.01.*, -0.17.*, 1.56.*, 0.002.*, 0.00.*, 42.00.*', csv))
        self.assertTrue(re.search('\n4.380.*, 0, Ego, -7.19.*, 171.42.*, -0.29.*, 1.56.*, 0.002.*, 0.00.*, 30.00.*', csv))
        self.assertTrue(re.search('\n4.380.*, 1, OverTaker, -4.22.*, 193.97.*, -0.33.*, 1.61.*, 0.002.*, 0.00.*, 42.00.*', csv))
        self.assertTrue(re.search('\n9.000.*, 0, Ego, -5.64.*, 310.01.*, -0.54.*, 1.55.*, 0.002.*, 0.00.*, 30.00.*', csv))
        self.assertTrue(re.search('\n9.000, 1, OverTaker, -4.019, 387.992, -0.697, 1.544, 0.002, 0.000, 42.000, 0.000, 5.575', csv))

    def test_routing(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/routing-test.xosc'), COMMON_ARGS)
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading routing-test.xosc', log)  is not None)
        self.assertTrue(re.search('.*Route::AddWaypoint Added waypoint 6: 261, 1, 50.00', log)  is not None)
        self.assertTrue(re.search('.*Route::AddWaypoint Added intermediate waypoint 7 roadId 260 laneId -1', log)  is not None)
        self.assertTrue(re.search('.*Route::AddWaypoint Added intermediate waypoint 11 roadId 220 laneId -1', log)  is not None)
        self.assertTrue(re.search('.*Route::AddWaypoint Added waypoint 12: 222, -1, 20.00', log)  is not None)
        self.assertTrue(re.search('\n25.46.* Route::AddWaypoint Added intermediate waypoint 3 roadId 280 laneId -1', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('\n25.46.* AquirePosition condition == true, distance 1.5. < tolerance \(2.00\), edge: rising', log)  is not None)
        self.assertTrue(re.search('\n25.46.: AquirePosition event complete after 1 execution', log)  is not None)
        self.assertTrue(re.search('\n38.31.* Stop condition == true, distance 1.85 < tolerance \(2.00\), edge: rising', log)  is not None)
        self.assertTrue(re.search('\n47.32.* QuitCondition timer expired at 4.00 seconds', log)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('\n7.00.*, 0, Ego, 291.875, 26.250, 0.000, 1.571, 0.00.*, 0.00.*, 50.00.*', csv))
        self.assertTrue(re.search('\n23.900, 0, Ego, 232.765, -3.753, 0.000, 6.063, 0.000, 0.000, 50.000.*', csv))
        self.assertTrue(re.search('\n43.300, 0, Ego, 639.136, -1.875, 0.000, 0.000, 0.000, 0.000, 0.100', csv))
        self.assertTrue(re.search('\n43.310, 0, Ego, 639.136, -1.875, 0.000, 0.000, 0.000, 0.000, 0.000', csv))

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
        self.assertTrue(re.search('^20.010: Brake2Condition == true, 20.0100 > 20.00 edge: rising', log, re.MULTILINE))
        self.assertTrue(re.search('^33.260: ActStopCondition == true, element: TargetBrake2Event state: END_TRANSITION, edge: rising', log, re.MULTILINE))

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('\n0.37.*, 0, Ego, 32.414, -1.535, 0.000, 0.000, 0.000, 0.000, 33.388', csv))
        self.assertTrue(re.search('\n0.37.*, 1, Target, 103.083, -1.535, 0.000, 0.000, 0.000, 0.000, 8.333', csv))
        self.assertTrue(re.search('\n4.89.*, 0, Ego, 122.579, -1.535, 0.000, 0.000, 0.000, 0.000, 10.180', csv))
        self.assertTrue(re.search('\n4.89.*, 1, Target, 140.750, -1.535, 0.000, 0.000, 0.000, 0.000, 8.333', csv))
        self.assertTrue(re.search('\n5.09.*, 0, Ego, 124.582, -1.535, 0.000, 0.000, 0.000, 0.000, 9.871', csv))
        self.assertTrue(re.search('\n5.090, 1, Target, 142.417, -1.519, 0.000, 0.056, 0.000, 0.000, 8.333', csv))

        self.assertTrue(re.search('\n7.710, 0, Ego, 153.805, -1.535, 0.000, 0.000, 0.000, 0.000, 9.360', csv))
        self.assertTrue(re.search('\n7.710, 1, Target, 164.250, 0.034, 0.000, 5.891, 0.000, 0.000, 8.333*', csv))
        self.assertTrue(re.search('\n16.71.*, 0, Ego, 192.299, -1.535, 0.000, 0.000, 0.000, 0.000, 0.100', csv))
        self.assertTrue(re.search('\n16.71.*, 1, Target, 200.38.*, -1.53.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*, 0.00.*', csv))
        self.assertTrue(re.search('\n25.210, 0, Ego, 241.946, -1.535, 0.000, 0.000, 0.000, 0.000, 5.277, 0.000, 5.813', csv))
        self.assertTrue(re.search('\n25.210, 1, Target, 255.164, -1.535, 0.000, 0.000, 0.000, 0.000, 5.000, 0.000, 3.503', csv))
        self.assertTrue(re.search('\n33.000, 0, Ego, 281.174, -1.535, 0.000, 0.000, 0.000, 0.000, 5.000, 0.000, 4.795', csv))
        self.assertTrue(re.search('\n33.000, 1, Target, 294.114, -1.535, 0.000, 0.000, 0.000, 0.000, 5.000, 0.000, 1.691', csv))

    def test_swarm(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/swarm.xosc'), COMMON_ARGS + '--seed 5')
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading swarm.xosc', log)  is not None)
        self.assertTrue(re.search('Using specified seed 5', log)  is not None)
        self.assertTrue(re.search('^0.00.*Ego New position:.*$\n^.*Pos\(10.20, 299.87, -0.53\) Rot\(1.56, 0.00, 0.00\) roadId 0 laneId -3 offset 0.00 t -8.00', log, re.MULTILINE))
        self.assertTrue(re.search('^0.00.*Init Ego TeleportAction standbyState -> startTransition -> runningState', log, re.MULTILINE))
        self.assertTrue(re.search('^0.00.*Init Ego LongitudinalAction runningState -> endTransition -> completeState', log, re.MULTILINE))

        # Check some scenario events
        self.assertTrue(re.search('^0.00.*: Swarm IR: 200.00, SMjA: 300.00, SMnA: 500.00, maxV: 75 vel: 30.00', log, re.MULTILINE))
        self.assertTrue(re.search('^60.01.*: SwarmStopTrigger == true, 60.0100 > 60.00 edge: none', log, re.MULTILINE))

        # Check vehicle key positions
        csv = generate_csv()

        self.assertTrue(re.search('^40.00.*, 0, Ego, 33.31.*, 699.01.*, -0.95.*, 1.45.*, 0.00.*, 0.00.*, 10.00.*', csv, re.MULTILINE))
        # Random generators differ on platforms => random traffic will be repeatable only per platform
        if platform == "win32":
            self.assertTrue(re.search('^5.000, 10, swarm10, 8.671, 152.427, -0.255, 1.564, 0.002, 0.000, 32.283', csv, re.MULTILINE))
            self.assertTrue(re.search('^20.000, 10, swarm10, 15.325, 480.049, -0.827, 1.523, 0.001, 0.000, 10.405', csv, re.MULTILINE))
            self.assertTrue(re.search('^20.000, 15, swarm15, 18.145, 579.901, -0.833, 1.491, 6.283, 0.000, 30.000', csv, re.MULTILINE))
            self.assertTrue(re.search('^20.000, 37, swarm37, -7.462, 397.500, -0.714, 4.684, 6.281, 0.000, 31.677', csv, re.MULTILINE))
        elif platform == "linux" or platform == "linux2":
            self.assertTrue(re.search('^5.000, 7, swarm7, 8.848, 177.565, -0.306, 1.563, 0.002, 0.000, 32.399', csv, re.MULTILINE))
            self.assertTrue(re.search('^20.000, 7, swarm7, 15.342, 480.389, -0.828, 1.523, 0.001, 0.000, 10.226', csv, re.MULTILINE))
            self.assertTrue(re.search('^20.000, 14, swarm14, 25.998, 586.691, -0.830, 1.488, 6.283, 0.000, 30.792', csv, re.MULTILINE))
            self.assertTrue(re.search('^20.000, 25, swarm25, -6.363, 258.622, -0.451, 4.700, 6.281, 0.000, 30.000', csv, re.MULTILINE))

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

        self.assertTrue(re.search('^5.540: Lane offset action 2 runningState -> endTransition -> completeState', log, re.MULTILINE))
        self.assertTrue(re.search('^9.00.*: Speed action 2 runningState -> endTransition -> completeState', log, re.MULTILINE))
        self.assertTrue(re.search('^13.00.*: Stop condition timer expired at 4.00 seconds', log, re.MULTILINE))
        self.assertTrue(re.search('^13.00.*: Stop condition == true, speed: 20.00 >= 20.00, edge: none', log, re.MULTILINE))

        # Check vehicle key positions
        csv = generate_csv()

        self.assertTrue(re.search('^4.010, 0, Ego, 36.039, -0.600, 0.000, 0.108, 0.000, 0.000, 8.030', csv, re.MULTILINE))
        self.assertTrue(re.search('^4.020, 0, Ego, 36.120, -0.600, 0.000, 0.000, 0.000, 0.000, 8.056', csv, re.MULTILINE))
        self.assertTrue(re.search('^4.030, 0, Ego, 36.200, -0.600, 0.000, 6.281, 0.000, 0.000, 8.083', csv, re.MULTILINE))
        self.assertTrue(re.search('^5.550, 0, Ego, 51.894, -1.535, 0.000, 0.000, 0.000, 0.000, 12.679', csv, re.MULTILINE))

    def test_follow_ghost(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/follow_ghost.xosc'), COMMON_ARGS)
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading follow_ghost.xosc', log)  is not None)
        
        # Check some scenario events
        self.assertTrue(re.search('^2.560.* SpeedEvent1 complete after 1 execution', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^3.010: LaneChangeCondition1 == true, 3.0100 > 3.00 edge: rising', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^19.020: EventStopCondition == true, element: StopEvent state: COMPLETE, edge: none', log, re.MULTILINE)  is not None)
        
        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^-1.000, 1, Ego_ghost, 8.211, 59.920, -0.057, 1.567, 0.002, 0.000, 9.950', csv, re.MULTILINE))
        self.assertTrue(re.search('^2.550, 0, Ego, 8.304, 82.995, -0.100, 1.567, 0.002, 6.283, 20.491, -0.000, 0.111', csv, re.MULTILINE))
        self.assertTrue(re.search('^2.560, 1, Ego_ghost, 8.519, 127.114, -0.198, 1.565, 0.002, 0.000, 27.750, 0.000, 0.503', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.500, 0, Ego, 5.700, 187.848, -0.325, 1.605, 0.002, 0.000, 27.778, -0.018, 4.643', csv, re.MULTILINE))
        self.assertTrue(re.search('^13.000, 1, Ego_ghost, 11.284, 358.924, -0.643, 1.549, 0.002, 0.000, 0.100, 0.000, 3.070', csv, re.MULTILINE))
        self.assertTrue(re.search('^14.570, 0, Ego, 11.133, 351.951, -0.629, 1.550, 0.002, 0.000, 0.033, -0.001, 2.770', csv, re.MULTILINE))

    def test_heading_trig(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'EnvironmentSimulator/Unittest/xosc/traj-heading-trig.xosc'), COMMON_ARGS)
        
        # Check some initialization steps
        self.assertTrue(re.search('.*traj-heading-trig.xosc', log)  is not None)
        
        # Check some scenario events
        self.assertTrue(re.search('^0.020: MyLaneChangeEvent standbyState -> startTransition -> runningState', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^1.310: MyAlignOrientationStartCondition == true, distance 0.00 < tolerance \(1.00\), rel orientation \[0.05, 0.00, 0.00\] \(tolerance 0.05\), edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^12.690: MyActStopCondition == true, distance 0.00 < tolerance \(1.00\), abs orientation \[0.95, 0.00, 0.00\] \(tolerance 0.05\), edge: none', log, re.MULTILINE)  is not None)
        
        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^1.310, 0, Car0, 438.194, -1.058, 0.000, 0.050, 0.000, 0.000, 13.889', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.320, 0, Car0, 438.194, -1.058, 0.000, 0.000, 0.000, 0.000, 13.889', csv, re.MULTILINE))
        self.assertTrue(re.search('^12.690, 0, Car0, 582.264, 41.303, 0.000, 0.951, 0.000, 0.000, 13.889', csv, re.MULTILINE))

    def test_lane_change_at_hw_exit(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'EnvironmentSimulator/Unittest/xosc/highway_exit.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*highway_exit.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('^3.380: position trigger == true, distance 1.70 < tolerance \(2.00\), edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^3.390: slowdown standbyState -> startTransition -> runningState', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^4.380: position trigger == true, distance 1.70 < tolerance \(2.00\), edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^4.380: Event slowdown event ended, overwritten by event lanechange event', log, re.MULTILINE)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^3.380, 0, Ego, 128.300, -4.500, 0.000, 0.000, 0.000, 0.000, 35.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^3.380, 1, Target, 148.300, -4.500, 0.000, 0.000, 0.000, 0.000, 35.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^4.390, 0, Ego, 163.650, -4.500, 0.000, 0.000, 0.000, 0.000, 35.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^4.390, 1, Target, 182.296, -4.500, 0.000, 0.000, 0.000, 0.000, 31.250', csv, re.MULTILINE))
        self.assertTrue(re.search('^7.000, 0, Ego, 255.000, -4.500, 0.000, 0.000, 0.000, 0.000, 35.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^7.000, 1, Target, 263.859, -6.691, 0.000, 6.250, 0.000, 0.000, 31.250', csv, re.MULTILINE))
        self.assertTrue(re.search('^11.500, 0, Ego, 412.500, -4.500, 0.000, 0.000, 0.000, 0.000, 35.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^11.500, 1, Target, 400.852, -30.959, 0.000, 5.933, 0.000, 0.000, 31.250', csv, re.MULTILINE))

    def test_lane_change_clothoid(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/lane-change_clothoid_based_trajectory.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*lane-change_clothoid_based_trajectory.xosc', log)  is not None)
        self.assertTrue(re.search('^Adding clothoid\(x=0.00 y=0.00 h=0.00 curv=0.01 curvDot=0.00 len=15.00 startTime=0.00 stopTime=0.00\)', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^Adding clothoid\(x=0.00 y=0.00 h=0.00 curv=0.00 curvDot=0.00 len=5.00 startTime=0.00 stopTime=0.00\)', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^Adding clothoid\(x=0.00 y=0.00 h=0.00 curv=-0.01 curvDot=0.00 len=15.00 startTime=0.00 stopTime=0.00\)', log, re.MULTILINE)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('^3.090: LaneChangeCondition2 == true, element: LaneChangeEvent1 state: COMPLETE, edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^3.450: LaneChangeCondition3 == true, element: LaneChangeEvent2 state: COMPLETE, edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^4.530: LaneChangeEvent3 runningState -> endTransition -> completeState', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^7.000: StopCondition timer 1.00s started', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^8.000: StopCondition == false, 7.0000 < 7.00 edge: falling', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^8.000: LaneChangeAct runningState -> endTransition -> completeState', log, re.MULTILINE)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^1.980, 0, Car, 77.500, -1.535, 0.000, 0.000, 0.000, 0.000, 13.889', csv, re.MULTILINE))
        self.assertTrue(re.search('^2.020, 0, Car, 78.056, -1.535, 0.000, 0.001, 0.000, 0.000, 13.889, 0.000, 4.761', csv, re.MULTILINE))
        self.assertTrue(re.search('^3.200, 0, Car, 94.371, -0.184, 0.000, 0.150, 0.000, 0.000, 13.889, 0.000, 1.320', csv, re.MULTILINE))
        self.assertTrue(re.search('^4.150, 0, Car, 107.473, 1.319, 0.000, 0.053, 0.000, 0.000, 13.889, 0.000, 1.320', csv, re.MULTILINE))
        self.assertTrue(re.search('^4.520, 0, Car, 112.609, 1.458, 0.000, 0.001, 0.000, 0.000, 13.889, 0.000, 3.436', csv, re.MULTILINE))

    def test_action_dynamics(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'EnvironmentSimulator/Unittest/xosc/test_action_dynamics.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*test_action_dynamics.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('2.010: LaneChange1Condition == true, 2.0100 > 2.00 edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('16.260: LaneChange2Event complete after 1 execution', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('21.520: LaneChange3Event complete after 1 execution', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('23.010: LaneOffset1Condition == true, 23.0100 > 23.00 edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('28.960: QuitCondition == true, element: LaneOffset1Event state: END_TRANSITION, edge: rising', log, re.MULTILINE)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^2.010, 0, Car, 30.100, 1.535, 0.000, 6.283, 0.000, 0.000, 10.000, 0.000, 0.880', csv, re.MULTILINE))
        self.assertTrue(re.search('^2.020, 0, Car, 30.200, 1.535, 0.000, 6.202, 0.000, 0.000, 10.000, 0.000, 1.166', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.370, 0, Car, 65.258, -2.006, 0.000, 6.121, 0.000, 0.000, 4.960, 0.000, 0.656', csv, re.MULTILINE))
        self.assertTrue(re.search('^7.010, 0, Car, 68.130, -2.527, 0.000, 6.083, 0.000, 0.000, 4.000, 0.000, 2.551', csv, re.MULTILINE))
        self.assertTrue(re.search('^7.020, 0, Car, 68.170, -2.535, 0.000, 0.000, 0.000, 0.000, 4.000, 0.000, 2.666', csv, re.MULTILINE))
        self.assertTrue(re.search('^14.400, 0, Car, 97.690, 0.177, 0.000, 0.154, 0.000, 0.000, 4.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^16.700, 0, Car, 106.890, 1.344, 0.000, 6.283, 0.000, 0.000, 4.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^17.820, 0, Car, 111.370, 1.126, 0.000, 6.152, 0.000, 0.000, 4.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^21.410, 0, Car, 131.415, -1.524, 0.000, 6.271, 0.000, 0.000, 7.000, 0.000, 1.239', csv, re.MULTILINE))
        self.assertTrue(re.search('^22.540, 0, Car, 139.325, -1.529, 0.000, 6.283, 0.000, 0.000, 7.000, 0.000, 4.990', csv, re.MULTILINE))
        self.assertTrue(re.search('^24.800, 0, Car, 155.145, 0.790, 0.000, 0.247, 0.000, 0.000, 7.000, 0.000, 6.207', csv, re.MULTILINE))
        self.assertTrue(re.search('^26.000, 0, Car, 163.545, 1.968, 0.000, 6.283, 0.000, 0.000, 7.000, 0.000, 5.074', csv, re.MULTILINE))

    def test_route_lane_change(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'EnvironmentSimulator/Unittest/xosc/route_lane_change.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*route_lane_change.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('Route::AddWaypoint Added waypoint 0: 2, -1, 100.00', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('Route::AddWaypoint Added intermediate waypoint 1 roadId 15 laneId -1', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('Route::AddWaypoint Added waypoint 2: 1, -1, 0.00', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('0.000: Pos\(22.47, 4.81, 0.00\) Rot\(4.89, 0.00, 0.00\) roadId 2 laneId -1 offset 0.00 t -1.75', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('31.300: LaneChangeCondition6 == true, element: LaneChangeEvent5 state: END_TRANSITION, edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('38.310: QuitCondition == true, element: LaneChangeEvent6 state: END_TRANSITION, edge: rising', log, re.MULTILINE)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^1.000, 0, Car, 28.935, -2.210, 0.000, 6.016, 0.000, 0.000, 10.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^3.140, 0, Car, 49.771, 1.806, 0.000, 0.243, 0.000, 0.000, 10.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^3.150, 0, Car, 49.770, 1.811, 0.000, 0.243, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^3.160, 0, Car, 49.770, 1.811, 0.000, 0.243, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^7.000, 0, Car, 27.534, 0.457, 0.000, 5.755, 0.000, 0.000, 10.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^8.600, 0, Car, 43.039, 2.396, 0.000, 0.225, 0.000, 0.000, 10.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^14.500, 0, Car, 20.277, -4.169, 0.000, 3.647, 0.000, 0.000, 10.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^16.500, 0, Car, 0.695, -8.146, 0.000, 3.287, 0.000, 0.000, 10.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^19.220, 0, Car, 29.876, -28.168, 0.000, 1.921, 0.000, 0.000, 10.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^23.000, 0, Car, 21.746, 8.737, 0.000, 1.753, 0.000, 0.000, 10.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^27.510, 0, Car, 16.477, -3.137, 0.000, 3.202, 0.000, 0.000, 10.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^35.050, 0, Car, 23.674, 1.513, 0.000, 5.207, 0.000, 0.000, 10.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^37.000, 0, Car, 41.309, -1.437, 0.000, 0.193, 0.000, 0.000, 10.000', csv, re.MULTILINE))

    def test_drop_bike(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/drop-bike.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*drop-bike.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('5.010: drop trigger == true, 5.0100 > 5.00 edge: none', log, re.MULTILINE)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^5.010, 1, Target, 225.300, -1.535, 0.000, 0.000, 0.000, 0.000, 30.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^5.010, 2, bike, 224.100, -2.135, 0.200, 1.570, 0.000, 0.000, 30.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^5.020, 2, bike, 224.100, -2.135, 0.100, 1.570, 0.000, 1.570, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^5.100, 2, bike, 224.100, -2.135, 0.100, 1.570, 0.000, 1.570, 0.000', csv, re.MULTILINE))

    def test_speed_over_distance(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'EnvironmentSimulator/Unittest/xosc/speed_over_distance.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*speed_over_distance.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('^10.080: SpeedChangeCondition2 == true, distance 0.93 < tolerance \(1.00\), edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^26.880: SpeedChangeCondition3 == true, distance 0.99 < tolerance \(1.00\), edge: none', log, re.MULTILINE)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^5.000, 0, Car, 31.740, -1.535, 0.000, 0.000, 0.000, 0.000, 10.928', csv, re.MULTILINE))
        self.assertTrue(re.search('^10.080, 0, Car, 100.069, -1.535, 0.000, 0.000, 0.000, 0.000, 13.889', csv, re.MULTILINE))
        self.assertTrue(re.search('^19.680, 0, Car, 180.014, -1.535, 0.000, 0.000, 0.000, 0.000, 2.778', csv, re.MULTILINE))
        self.assertTrue(re.search('^28.000, 0, Car, 203.527, -1.535, 0.000, 0.000, 0.000, 0.000, 3.814', csv, re.MULTILINE))
        self.assertTrue(re.search('^34.860, 0, Car, 299.667, -1.535, 0.000, 0.000, 0.000, 0.000, 22.222', csv, re.MULTILINE))

    def test_collision_condition1(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'EnvironmentSimulator/Unittest/xosc/test-collision-detection.xosc'), COMMON_ARGS + \
            '--disable_controllers')

        # Check some initialization steps
        self.assertTrue(re.search('.*test-collision-detection.xosc', log)  is not None)

        # Check some scenario events
        self.assertFalse(re.search('^6.260: Collision between Ego and NPC1', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^5.250: collision 0 between Ego and NPC2', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^6.260: collision 0 between Ego and NPC1', log, re.MULTILINE)  is not None)

        # Check vehicle key positions
        csv = generate_csv()

        self.assertTrue(re.search('^20.000, 0, Ego, 110.000, -0.035, 0.000, 0.000, 0.000, 0.000, 5.000, 0.000, 2.971', csv, re.MULTILINE))
        self.assertTrue(re.search('^20.000, 1, NPC1, 60.000, -1.535, 0.000, 0.000, 0.000, 0.000, 1.000, 0.000, 0.594', csv, re.MULTILINE))
        self.assertTrue(re.search('^20.000, 2, NPC2, 30.000, 1.535, 0.000, 3.142, 0.000, 0.000, 1.000, 0.000, 0.594', csv, re.MULTILINE))

    def test_collision_condition2(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'EnvironmentSimulator/Unittest/xosc/test-collision-detection.xosc'), COMMON_ARGS + \
            '--disable_controllers --collision')
        
        # Same as previous, but confirms explicit collision  detection in condition when global collision detection is disabled

        # Check some initialization steps
        self.assertTrue(re.search('.*test-collision-detection.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('^5.250: Collision between Ego and NPC2', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^5.250: collision 0 between Ego and NPC2', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^6.260: Collision between Ego and NPC1', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^6.260: collision 0 between Ego and NPC1', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^7.090: Collision between Ego and NPC2 dissolved', log, re.MULTILINE)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^20.000, 0, Ego, 110.000, -0.035, 0.000, 0.000, 0.000, 0.000, 5.000, 0.000, 2.971', csv, re.MULTILINE))
        self.assertTrue(re.search('^20.000, 1, NPC1, 60.000, -1.535, 0.000, 0.000, 0.000, 0.000, 1.000, 0.000, 0.594', csv, re.MULTILINE))
        self.assertTrue(re.search('^20.000, 2, NPC2, 30.000, 1.535, 0.000, 3.142, 0.000, 0.000, 1.000, 0.000, 0.594', csv, re.MULTILINE))

    def test_add_delete_entity(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'EnvironmentSimulator/Unittest/xosc/add_delete_entity.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*add_delete_entity.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('^2.000: Failed to activate obj Car1. Already active \(1 instances in active list\)', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^3.010: Added entity Car2', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^7.010: Deleted entity Car2', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^9.010: Failed to deactivate obj Car2. Already inactive \(0 in active list\)', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^10.010: Deleted entity Car1', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^12.010: Added entity Car1', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^15.900: Deleted entity Car1', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^17.920: StopCondition timer expired at 2.00 seconds', log, re.MULTILINE)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^3.020, 0, Car1, 68.722, -1.535, 0.000, 0.000, 0.000, 0.000, 19.444', csv, re.MULTILINE))
        self.assertTrue(re.search('^3.020, 1, Car2, 90.194, -1.535, 0.000, 0.000, 0.000, 0.000, 19.444', csv, re.MULTILINE))
        self.assertTrue(re.search('^7.010, 0, Car1, 146.306, -1.535, 0.000, 0.000, 0.000, 0.000, 19.444', csv, re.MULTILINE))
        self.assertTrue(re.search('^7.010, 1, Car2, 167.778, -1.535, 0.000, 0.000, 0.000, 0.000, 19.444', csv, re.MULTILINE))
        self.assertTrue(re.search('^7.010, 0, Car1, 146.306, -1.535, 0.000, 0.000, 0.000, 0.000, 19.444', csv, re.MULTILINE))
        self.assertTrue(re.search('^15.880, 0, Car1, 499.889, -1.535, 0.000, 0.000, 0.000, 0.000, 19.444', csv, re.MULTILINE))
        self.assertTrue(re.search('^15.890, 0, Car1, 500.000, -1.535, 0.000, 0.000, 0.000, 0.000, 19.444', csv, re.MULTILINE))
        
    def test_multi_lane_changes(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'EnvironmentSimulator/Unittest/xosc/multi_lane_changes.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*multi_lane_changes.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('^0.010: act_start == true, 0.0100 > 0.00 edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^1.140: start_trigger1 == true, rel_dist: 0.08 < 0.10, edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^3.350: start_trigger2 == true, rel_dist: 0.08 < 0.10, edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^7.360: start_trigger3 timer expired at 1.00 seconds', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^15.330: start_trigger6 == true, HWT: 1.00 > 1.00, edge none', log, re.MULTILINE)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^5.500, 0, Ego, 109.028, -6.000, 0.000, 0.000, 0.000, 0.000, 15.278', csv, re.MULTILINE))
        self.assertTrue(re.search('^5.500, 1, Target1, 119.379, -6.000, 0.000, 0.000, 0.000, 0.000, 15.278', csv, re.MULTILINE))
        self.assertTrue(re.search('^5.500, 2, Target2, 129.028, -6.000, 0.000, 0.000, 0.000, 0.000, 15.278', csv, re.MULTILINE))
        self.assertTrue(re.search('^5.500, 3, Target3, 141.024, -6.000, 0.000, 0.000, 0.000, 0.000, 16.822', csv, re.MULTILINE))
        self.assertTrue(re.search('^12.200, 0, Ego, 211.389, -6.000, 0.000, 0.000, 0.000, 0.000, 15.278, 0.000, 4.752', csv, re.MULTILINE))
        self.assertTrue(re.search('^12.200, 1, Target1, 248.303, -9.250, 0.000, 0.108, 0.000, 0.000, 22.562, 0.000, 1.134', csv, re.MULTILINE))
        self.assertTrue(re.search('^12.200, 2, Target2, 235.142, -2.071, 0.000, 0.040, 0.000, 0.000, 20.672, 0.000, 3.062', csv, re.MULTILINE))
        self.assertTrue(re.search('^12.200, 3, Target3, 243.827, -6.000, 0.000, 0.000, 0.000, 0.000, 15.278, 0.000, 0.729', csv, re.MULTILINE))
        self.assertTrue(re.search('^20.000, 0, Ego, 330.556, -6.000, 0.000, 0.000, 0.000, 0.000, 15.278, 0.000, 5.936', csv, re.MULTILINE))
        self.assertTrue(re.search('^20.000, 1, Target1, 374.987, -6.000, 0.000, 0.000, 0.000, 0.000, 15.278, 0.000, 4.793', csv, re.MULTILINE))
        self.assertTrue(re.search('^20.000, 2, Target2, 390.316, -6.000, 0.000, 0.000, 0.000, 0.000, 15.278, 0.000, 0.156', csv, re.MULTILINE))
        self.assertTrue(re.search('^20.000, 3, Target3, 362.993, -6.000, 0.000, 0.000, 0.000, 0.000, 15.278, 0.000, 1.913', csv, re.MULTILINE))

    def test_init_cases(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'EnvironmentSimulator/Unittest/xosc/init_test.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*init_test.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('^-2.000: Ego New position:\n-2.000: Pos\(200.00, -1.53, 0.00\) '\
            'Rot\(0.00, 0.00, 0.00\) roadId 1 laneId -1 offset 0.00 t -1.53', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^-2.000: OverTaker2 New position:\n-2.000: Pos\(280.00, -1.53, 0.00\) '\
            'Rot\(0.00, 0.00, 0.00\) roadId 1 laneId -1 offset 0.00 t -1.53', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^-2.000: Ego_ghost New position:\n-2.000: Pos\(200.00, -1.53, 0.00\) '\
            'Rot\(0.00, 0.00, 0.00\) roadId 1 laneId -1 offset 0.00 t -1.53', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^-2.000: Init OverTaker1 LongitudinalAction runningState -> endTransition -> completeState', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^0.990: Init OverTaker1 RoutingAction runningState -> endTransition -> completeState', log, re.MULTILINE)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^-2.000, 0, Ego, 200.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^-2.000, 1, OverTaker1, 250.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^-2.000, 2, OverTaker2, 280.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^-2.000, 3, Ego_ghost, 200.000, -1.535, 0.000, 0.102, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^-1.990, 0, Ego, 200.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^-1.990, 1, OverTaker1, 250.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^-1.990, 2, OverTaker2, 280.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^-1.990, 3, Ego_ghost, 200.100, -1.535, 0.000, 0.102, 0.000, 0.000, 10.000, 0.000, 0.286', csv, re.MULTILINE))
        self.assertTrue(re.search('^-1.980, 0, Ego, 200.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^-1.980, 1, OverTaker1, 250.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^-1.980, 2, OverTaker2, 280.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^-1.980, 3, Ego_ghost, 200.200, -1.525, 0.000, 0.102, 0.000, 0.000, 10.000, 0.000, 0.571', csv, re.MULTILINE))

        self.assertTrue(re.search('^-0.010, 0, Ego, 200.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^-0.010, 1, OverTaker1, 250.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^-0.010, 2, OverTaker2, 280.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^-0.010, 3, Ego_ghost, 219.900, 0.491, 0.000, 0.102, 0.000, 0.000, 10.000, 0.000, 0.308', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 0, Ego, 200.100, -1.533, 0.000, 0.002, 0.000, 0.000, 10.000, 0.101, 0.286', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 1, OverTaker1, 250.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 2, OverTaker2, 280.100, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 3, Ego_ghost, 220.000, 0.501, 0.000, 0.102, 0.000, 0.000, 10.000, 0.000, 0.594', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.010, 0, Ego, 200.200, -1.532, 0.000, 0.004, 0.000, 0.000, 10.000, 0.100, 0.571', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.010, 1, OverTaker1, 250.100, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.286', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.010, 2, OverTaker2, 280.200, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.286', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.010, 3, Ego_ghost, 220.100, 0.512, 0.000, 0.102, 0.000, 0.000, 10.000, 0.000, 0.880', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.020, 0, Ego, 200.300, -1.530, 0.000, 0.006, 0.000, 0.000, 10.000, 0.099, 0.857', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.020, 1, OverTaker1, 250.200, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.571', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.020, 2, OverTaker2, 280.300, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.571', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.020, 3, Ego_ghost, 220.200, 0.522, 0.000, 0.102, 0.000, 0.000, 10.000, 0.000, 1.166', csv, re.MULTILINE))

    def test_reverse_lane_change(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'EnvironmentSimulator/Unittest/xosc/reverse_lane_change.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*reverse_lane_change.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('^1.010: reverse_trigger == true, 1.0100 > 1.00 edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^5.010: lateral_event_trigger == true, 5.0100 > 5.00 edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^5.370: reverse_event complete after 1 execution', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^8.010: lateral_event complete after 1 execution', log, re.MULTILINE)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^2.000, 0, Ego, 276.626, -1.535, 0.000, 0.000, 0.000, 0.000, 10.507, 0.000, 0.675', csv, re.MULTILINE))
        self.assertTrue(re.search('^3.190, 0, Ego, 283.248, -1.535, 0.000, 0.000, 0.000, 0.000, 0.017, 0.000, 0.748', csv, re.MULTILINE))
        self.assertTrue(re.search('^3.200, 0, Ego, 283.248, -1.535, 0.000, 0.000, 0.000, 0.000, -0.083, 0.000, 0.745', csv, re.MULTILINE))
        self.assertTrue(re.search('^7.000, 0, Ego, 241.296, 0.712, 0.000, 6.184, 0.000, 0.000, -13.889, 0.000, -6.019', csv, re.MULTILINE))
        self.assertTrue(re.search('^10.000, 0, Ego, 199.629, 1.535, 0.000, 0.000, 0.000, 0.000, -13.889, 0.000, -5.686', csv, re.MULTILINE))

    def test_ghost_restart(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'EnvironmentSimulator/Unittest/xosc/ghost_restart.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*ghost_restart.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('^-0.500: Pos\(50.00, -11.70, 0.00\) Rot\(0.00, 0.00, 0.00\) roadId 1 laneId -4 offset 0.00 t -11.70', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^1.000: Set parameter myparam = true', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^1.010: parameter myparam 0 == true edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^1.010: Pos\(60.20, -11.70, 0.00\) Rot\(0.00, 0.00, 0.00\) roadId 1 laneId -4 offset 0.00 t -11.70', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^0.520: AddedGhostTeleport standbyState -> endTransition -> completeState', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^2.520: newspeed runningState -> endTransition -> completeState', log, re.MULTILINE)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^-0.500, 0, Ego, 50.000, -11.700, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^-0.500, 1, Ego_ghost, 50.000, -11.700, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.010, 1, Ego_ghost, 65.100, -11.700, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 5.444', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.020, 1, Ego_ghost, 65.300, -10.775, 0.000, 0.183, 0.000, 0.000, 10.000, 0.000, 1.166', csv, re.MULTILINE))
        self.assertTrue(re.search('^2.000, 0, Ego, 69.963, -10.169, 0.000, 0.221, 0.000, 0.000, 10.000, 0.018, 0.880', csv, re.MULTILINE))
        self.assertTrue(re.search('^2.000, 1, Ego_ghost, 75.100, -8.962, 0.000, 0.183, 0.000, 0.000, 10.000, 0.000, 4.033', csv, re.MULTILINE))
        self.assertTrue(re.search('^5.000, 0, Ego, 99.733, -8.019, 0.000, 6.272, 0.000, 0.000, 10.000, 0.016, 4.913', csv, re.MULTILINE))
        self.assertTrue(re.search('^5.000, 1, Ego_ghost, 105.100, -8.000, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 1.783', csv, re.MULTILINE))

if __name__ == "__main__":
    # execute only if run as a script

    # Run next line instead to execute only one test
    # unittest.main(argv=['ignored', '-v', 'TestSuite.test_ghost_restart'])
    
    unittest.main(verbosity=2)
