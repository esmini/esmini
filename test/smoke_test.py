import re
import sys
from sys import platform
from test_common import *
import unittest

ESMINI_PATH = '../'
COMMON_ARGS = '--headless --fixed_timestep 0.01 --record sim.dat '


class TestSuite(unittest.TestCase):

    def test_cut_in(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/cut-in.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*cut-in.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('0.010.* CutInActStart == true, 0.0100 > 0.00 edge: none', log)  is not None)
        self.assertTrue(re.search('\\n7.740: BrakeCondition_HWT_0.7 == true, HWT: 0.70 > 0.70, edge rising', log)  is not None)
        self.assertTrue(re.search('\\n21.[678].* StopCondition timer expired at 5.00 seconds', log)  is not None)

    def test_ltap_od(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/ltap-od.xosc'), COMMON_ARGS \
            + '--disable_controllers')

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*ltap-od.xosc', log)  is not None)
        self.assertTrue(re.search('.*Route::AddWaypoint Added intermediate waypoint 1 roadId 15 laneId -1', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('\\n5.500: Synchronize masterTimeToDest \\(0.003\\) reached within this timestep \\(0.010\\)', log)  is not None)
        self.assertTrue(re.search('\\n9.5.* QuitCondition timer expired at 4.0. seconds', log)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('\\n6.500, 0, Ego, 28.542, -7.876, 0.000, 1.779, 0.000, 0.000, 10.000, -0.006, 3.502', csv))
        self.assertTrue(re.search('\\n6.500, 1, NPC, 24.456, 0.305, 0.000, 5.394, 0.000, 0.000, 7.000, 0.335, 5.147', csv))

    def test_variables(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'EnvironmentSimulator/Unittest/xosc/lane_change_trig_by_variable.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*lane_change_trig_by_variable.xosc', log)  is not None)
        self.assertTrue(re.search('\\n3.010: Lane change Target condition by variable == true, 3.0100 > 3.00 edge: rising', log)  is not None)

        csv = generate_csv()
        self.assertTrue(re.search('\\n0.500, 0, Truck, 55.800, -47.582, 0.000, 6.170, 0.000, 0.000, 20.000, 0.000, 3.439', csv))
        self.assertTrue(re.search('4.000, 0, Truck, 125.471, -53.480, 0.000, 0.035, 0.000, 0.000, 20.000, 0.002, 2.377\\n', csv))
        self.assertTrue(re.search('4.000, 1, Truck\\+, 119.470, -53.503, 0.000, 0.008, 0.000, 0.000, 0.000, 0.013, 2.377\\n', csv))
        self.assertTrue(re.search('\\n7.000, 0, Truck, 185.008, -59.267, 0.000, 6.051, 0.000, 0.000, 20.000, -0.001, 4.159', csv))

    def test_trajectory(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/trajectory-test.xosc'), COMMON_ARGS \
            + '--disable_controllers')

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*trajectory-test.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('\\n8.00.*FollowTrajectoryClothoidTrigger == true, element: FollowTrajectoryPLineEvent state: END_TRANSITION', log)  is not None)
        self.assertTrue(re.search('\\n24.21.* FollowTrajectoryNurbsAction runningState -> endTransition -> completeState', log)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('\\n4.050, 1, Target, 129.062, 13.848, -3.447, 0.539, 0.021, 0.003, 29.319, 0.065, 0.323', csv))
        self.assertTrue(re.search('\\n4.100.*, 0, Ego, 115.042, 4.864, -3.006, 0.281, 0.032, 0.000, 16.000', csv))
        self.assertTrue(re.search('\\n4.100, 1, Target, 130.300, 14.620, -3.477, 0.558, 0.020, 0.003, 29.065, 0.050, 4.489', csv))
        self.assertTrue(re.search('\\n11.100.*, 0, Ego, 200.713, 72.600, -2.443, 1.057, 6.263, 0.000, 16.000', csv))
        self.assertTrue(re.search('\\n11.100, 1, Target, 206.003, 66.438, -2.496, 2.508, 6.281, 6.263, 17.500, -0.253, 1.548', csv))
        self.assertTrue(re.search('\\n17.250.*, 0, Ego, 217.345, 167.663, 1.989, 1.738, 6.209, 0.000, 16.000', csv))
        self.assertTrue(re.search('\\n17.250, 1, Target, 210.632, 157.507, 1.295, 1.225, 6.216, 0.032, 14.907, 0.050, 2.996', csv))
        self.assertTrue(re.search('\\n25.000.*, 0, Ego, 206.081, 288.506, 5.436, 1.188, 6.238, 0.000, 16.000', csv))
        self.assertTrue(re.search('\\n25.000, 1, Target, 216.288, 307.524, 6.706, 0.968, 6.214, 0.000, 21.101, -0.032, 5.799', csv))

    def test_synchronize(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/synchronize.xosc'), COMMON_ARGS \
            + '--disable_controllers')

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*synchronize.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('9.940: Synchronize masterTimeToDest \\(0.006\\) reached within this timestep \\(0.010\\)', log)  is not None)
        self.assertTrue(re.search('\\n9.9.* Synchronize_NPC_Event complete after 1 execution', log)  is not None)
        self.assertTrue(re.search('\\n19.74.* Free_Speed_Condition_NPC == true, distance 4.81 < tolerance \\(5.00\\), edge: rising', log)  is not None)
        self.assertTrue(re.search('\\n19.74.* Triggering entity 0: Ego', log)  is not None)
        self.assertTrue(re.search('\\n32.170: storyBoard runningState -> stopTransition -> completeState', log)  is not None)

        # Check vehicle key positions
        csv = generate_csv()

        self.assertTrue(re.search('\\n10.000.*, 0, Ego, 10.20.*, 299.95.*, -0.52.*, 1.55.*, 0.00.*, 0.00.*, 20.00.*', csv))
        self.assertTrue(re.search('\\n10.000.*, 1, NPC1, 6.70.*, 305.00.*, -0.53.*, 1.55.*, 0.00.*, 0.00.*, 20.00.*', csv))
        self.assertTrue(re.search('\\n10.000.*, 2, NPC2, 9.98.*, 284.96.*, -0.49.*, 1.55.*, 0.00.*, 0.00.*, 20.00.*', csv))
        self.assertTrue(re.search('\\n10.000.*, 3, NPC3, 13.85.*, 296.90.*, -0.52.*, 1.55.*, 0.00.*, 0.00.*, 20.00.*', csv))
        self.assertTrue(re.search('\\n10.000.*, 4, NPC4, 10.70.*, 329.92.*, -0.58.*, 1.55.*, 0.00.*, 0.00.*, 20.00.*', csv))
        self.assertTrue(re.search('\\n10.000.*, 5, MS1, 17.27.*, 299.76.*, -0.52.*, 1.55.*, 0.00.*, 0.00.*, 0.00.*', csv))
        self.assertTrue(re.search('\\n10.000.*, 6, MS2, 23.37.*, 499.07.*, -0.84.*, 1.51.*, 0.00.*, 0.00.*, 0.00.*', csv))

        self.assertTrue(re.search('\\n25.000, 1, NPC1, 22.644, 630.990, -0.831, 1.476, 0.001, 0.000, 30.000', csv))
        self.assertTrue(re.search('\\n25.000, 2, NPC2, 20.598, 565.401, -0.839, 1.495, 6.283, 0.000, 9.928', csv))
        self.assertTrue(re.search('\\n25.000, 3, NPC3, 25.708, 583.162, -0.832, 1.489, 6.283, 0.000, 17.000, -0.001, 3.212', csv))
        self.assertTrue(re.search('\\n25.000, 4, NPC4, 24.312, 610.213, -0.826, 1.481, 6.283, 0.000, 9.928', csv))

    def test_left_hand_by_heading(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/left-hand-traffic_by_heading.xosc'), COMMON_ARGS \
            + '--disable_controllers')

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*left-hand-traffic_by_heading.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('\\n3.75.* Lane change == true, rel_dist: 10.02 > 10.00, edge: rising', log)  is not None)
        self.assertTrue(re.search('\\n5.74.* Lane change complete after 1 execution', log)  is not None)
        self.assertTrue(re.search('\\n9.75.* QuitCondition timer expired at 4.00 seconds', log)  is not None)
        self.assertTrue(re.search('\\n9.75.* storyBoard runningState -> stopTransition -> completeState', log)  is not None)

        # Check vehicle key positions
        csv = generate_csv()

        self.assertTrue(re.search('\\n2.500, 0, Ego, -7.546, 115.028, -0.170, 1.566, 0.002, 6.283, 30.000, -0.000, 0.657', csv))
        self.assertTrue(re.search('\\n2.500, 1, OverTaker, -3.971, 115.014, -0.170, 1.566, 0.002, 0.000, 42.000, -0.000, 4.690', csv))
        self.assertTrue(re.search('\\n4.380, 0, Ego, -7.198, 171.427, -0.293, 1.563, 0.002, 6.283, 30.000, -0.000, 4.721', csv))
        self.assertTrue(re.search('\\n4.380, 1, OverTaker, -4.253, 193.963, -0.336, 1.618, 0.002, 0.000, 42.000, 0.004, 4.096', csv))
        self.assertTrue(re.search('\\n9.000, 1, OverTaker, -4.021, 387.927, -0.697, 1.544, 0.002, 6.283, 42.000, -0.001, 5.575', csv))
        self.assertTrue(re.search('\\n9.010, 0, Ego, -5.639, 310.318, -0.547, 1.555, 0.002, 6.283, 30.000, -0.000, 5.737', csv))

    def test_left_hand_using_road_rule(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/left-hand-traffic_using_road_rule.xosc'), COMMON_ARGS \
            + '--disable_controllers')

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*left-hand-traffic_using_road_rule.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('\\n3.75.* Lane change == true, rel_dist: 10.02 > 10.00, edge: rising', log)  is not None)
        self.assertTrue(re.search('\\n5.74.* Lane change complete after 1 execution', log)  is not None)
        self.assertTrue(re.search('\\n9.75.* QuitCondition timer expired at 4.00 seconds', log)  is not None)
        self.assertTrue(re.search('\\n9.75.* storyBoard runningState -> stopTransition -> completeState', log)  is not None)

        # Check vehicle key positions
        csv = generate_csv()

        self.assertTrue(re.search('\\n2.500, 0, Ego, -7.546, 115.028, -0.170, 1.566, 0.002, 0.000, 30.000, -0.000, 0.657', csv))
        self.assertTrue(re.search('\\n2.500, 1, OverTaker, -3.971, 115.014, -0.170, 1.566, 0.002, 0.000, 42.000, -0.000, 4.690', csv))
        self.assertTrue(re.search('\\n4.380, 0, Ego, -7.198, 171.427, -0.293, 1.563, 0.002, 0.000, 30.000, -0.000, 4.721', csv))
        self.assertTrue(re.search('\\n4.380, 1, OverTaker, -4.253, 193.963, -0.336, 1.618, 0.002, 0.000, 42.000, 0.004, 4.096', csv))
        self.assertTrue(re.search('\\n9.000, 1, OverTaker, -4.021, 387.927, -0.697, 1.544, 0.002, 6.283, 42.000, -0.001, 5.575', csv))
        self.assertTrue(re.search('\\n9.010, 0, Ego, -5.639, 310.318, -0.547, 1.555, 0.002, 0.000, 30.000, -0.000, 5.737', csv))

    def test_routing(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/routing-test.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*routing-test.xosc', log)  is not None)
        self.assertTrue(re.search('.*Route::AddWaypoint Added waypoint 6: 261, 1, 50.00', log)  is not None)
        self.assertTrue(re.search('.*Route::AddWaypoint Added intermediate waypoint 7 roadId 260 laneId -1', log)  is not None)
        self.assertTrue(re.search('.*Route::AddWaypoint Added intermediate waypoint 11 roadId 220 laneId -1', log)  is not None)
        self.assertTrue(re.search('.*Route::AddWaypoint Added waypoint 12: 222, -1, 20.00', log)  is not None)
        self.assertTrue(re.search('\\n25.46.* Route::AddWaypoint Added intermediate waypoint 3 roadId 280 laneId -1', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('\\n25.46.* AquirePosition condition == true, distance 1.5. < tolerance \\(2.00\\), edge: rising', log)  is not None)
        self.assertTrue(re.search('\\n25.46.: AquirePosition event complete after 1 execution', log)  is not None)
        self.assertTrue(re.search('\\n38.31.* Stop condition == true, distance 1.85 < tolerance \\(2.00\\), edge: rising', log)  is not None)
        self.assertTrue(re.search('\\n47.31.* QuitCondition timer expired at 4.00 seconds', log)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('\\n7.00.*, 0, Ego, 291.875, 26.250, 0.000, 1.571, 0.00.*, 0.00.*, 50.00.*', csv))
        self.assertTrue(re.search('\\n23.900, 0, Ego, 232.765, -3.753, 0.000, 6.066, 0.000, 0.000, 50.000.*', csv))
        self.assertTrue(re.search('\\n43.300, 0, Ego, 639.136, -1.875, 0.000, 0.000, 0.000, 0.000, 0.100', csv))
        self.assertTrue(re.search('\\n43.310, 0, Ego, 639.136, -1.875, 0.000, 0.000, 0.000, 0.000, 0.000', csv))

    def test_acc(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/acc-test.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*acc-test.xosc', log)  is not None)
        self.assertTrue(re.search('.*Ego New position:.*$\\n^.*Pos\\(20.00, -1.53, 0.00\\) Rot\\(0.00, 0.00, 0.00\\) roadId 1 laneId -1 offset 0.00 t -1.53', log, re.MULTILINE))
        self.assertTrue(re.search('.*Target New position:.*$\\n^.*Pos\\(100.00, -1.53, 0.00\\) Rot\\(0.00, 0.00, 0.00\\) roadId 1 laneId -1 offset 0.00 t -1.53', log, re.MULTILINE))

        # Check some scenario events
        self.assertTrue(re.search('^5.010: LaneChange1Condition == true, 5.0100 > 5.00 edge: rising', log, re.MULTILINE))
        self.assertTrue(re.search('^7.010: LaneChange2Condition == true, 7.0100 > 7.00 edge: rising', log, re.MULTILINE))
        self.assertTrue(re.search('^11.010: BrakeCondition == true, 11.0100 > 11.00 edge: rising', log, re.MULTILINE))
        self.assertTrue(re.search('^17.010: BrakeCondition == true, 17.0100 > 17.00 edge: rising', log, re.MULTILINE))
        self.assertTrue(re.search('^20.010: Brake2Condition == true, 20.0100 > 20.00 edge: rising', log, re.MULTILINE))
        self.assertTrue(re.search('^33.260: StopCondition == true, element: TargetBrake2Event state: END_TRANSITION, edge: rising', log, re.MULTILINE))

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('\\n0.37.*, 0, Ego, 32.414, -1.535, 0.000, 0.000, 0.000, 0.000, 33.388', csv))
        self.assertTrue(re.search('\\n0.37.*, 1, Target, 103.083, -1.535, 0.000, 0.000, 0.000, 0.000, 8.333', csv))
        self.assertTrue(re.search('\\n4.89.*, 0, Ego, 122.579, -1.535, 0.000, 0.000, 0.000, 0.000, 10.180', csv))
        self.assertTrue(re.search('\\n4.89.*, 1, Target, 140.750, -1.535, 0.000, 0.000, 0.000, 0.000, 8.333', csv))
        self.assertTrue(re.search('\\n5.09.*, 0, Ego, 124.582, -1.535, 0.000, 0.000, 0.000, 0.000, 9.871', csv))
        self.assertTrue(re.search('\\n5.090, 1, Target, 142.416, -1.514, 0.000, 0.064, 0.000, 0.000, 8.333, 0.040, 1.810', csv))

        self.assertTrue(re.search('\\n7.710, 0, Ego, 153.667, -1.535, 0.000, 0.000, 0.000, 0.000, 9.196, 0.000, 4.915', csv))
        self.assertTrue(re.search('\\n7.710, 1, Target, 163.513, -0.000, 0.000, 5.891, 0.000, 0.000, 8.333, -0.055, 1.359', csv))
        self.assertTrue(re.search('\\n16.710, 0, Ego, 191.301, -1.535, 0.000, 0.000, 0.000, 0.000, 0.103, 0.000, 5.627', csv))
        self.assertTrue(re.search('\\n16.710, 1, Target, 199.394, -1.535, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 4.082', csv))
        self.assertTrue(re.search('\\n25.210, 0, Ego, 240.952, -1.535, 0.000, 0.000, 0.000, 0.000, 5.277, 0.000, 2.972', csv))
        self.assertTrue(re.search('\\n25.210, 1, Target, 254.169, -1.535, 0.000, 0.000, 0.000, 0.000, 5.000, 0.000, 3.503', csv))
        self.assertTrue(re.search('\\n33.000, 0, Ego, 280.179, -1.535, 0.000, 0.000, 0.000, 0.000, 5.000, 0.000, 1.954', csv))
        self.assertTrue(re.search('\\n33.000, 1, Target, 293.119, -1.535, 0.000, 0.000, 0.000, 0.000, 5.000, 0.000, 1.691', csv))

    def test_swarm(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/swarm.xosc'), COMMON_ARGS + ' --seed 2426643349' + ' --fixed_timestep 0.05')

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*swarm.xosc', log)  is not None)
        self.assertTrue(re.search('Using specified seed 2426643349', log)  is not None)
        self.assertTrue(re.search('^0.00.*Ego New position:.*$\\n^.*Pos\\(10.20, 299.87, -0.53\\) Rot\\(1.56, 0.00, 0.00\\) roadId 0 laneId -3 offset 0.00 t -8.00', log, re.MULTILINE))
        self.assertTrue(re.search('^0.00.*Init Ego TeleportAction initState -> startTransition -> runningState', log, re.MULTILINE))
        self.assertTrue(re.search('^0.00.*Init Ego LongitudinalAction runningState -> endTransition -> completeState', log, re.MULTILINE))

        # Check some scenario events
        self.assertTrue(re.search('^0.00.*: Swarm IR: 200.00, SMjA: 300.00, SMnA: 500.00, maxV: 75 vel: 30.00', log, re.MULTILINE))
        self.assertTrue(re.search('^60.05.*: SwarmStopTrigger == true, 60.0500 > 60.00 edge: none', log, re.MULTILINE))

        # Check vehicle key positions
        csv = generate_csv()

        self.assertTrue(re.search('^40.00.*, 0, Ego, 33.31.*, 699.01.*, -0.95.*, 1.45.*, 0.00.*, 0.00.*, 10.00.*', csv, re.MULTILINE))
        # Random generators differ on platforms => random traffic will be repeatable only per platform
        if platform == "win32":
            self.assertTrue(re.search('^5.000, 0, Ego, 11.090, 349.861, -0.625, 1.550, 0.002, 0.000, 10.000, -0.000, 4.627', csv, re.MULTILINE))
            self.assertTrue(re.search('^5.000, 1, swarm_0, 12.730, 199.986, -0.348, 1.562, 0.002, 0.000, 30.000, -0.000, 1.315', csv, re.MULTILINE))
            self.assertTrue(re.search('^11.850, 51, swarm_30, 12.804, 208.221, -0.363, 1.562, 0.002, 0.000, 30.481, -0.000, 5.765', csv, re.MULTILINE))
            self.assertTrue(re.search('^11.850, 52, swarm_30\\+, 12.751, 202.221, -0.352, 1.562, 0.002, 0.000, 30.000, -0.000, 5.700', csv, re.MULTILINE))
            self.assertTrue(re.search('^11.850, 53, swarm_30\\+\\+, 12.700, 196.222, -0.341, 1.562, 0.002, 0.000, 30.000, -0.000, 5.700', csv, re.MULTILINE))
            self.assertTrue(re.search('^11.850, 54, swarm_30\\+\\+\\+, 12.645, 189.522, -0.328, 1.563, 0.002, 0.000, 30.000, -0.000, 5.700', csv, re.MULTILINE))
            self.assertTrue(re.search('^20.000, 0, Ego, 16.330, 499.764, -0.841, 1.517, 0.000, 0.000, 10.000, -0.001, 5.942', csv, re.MULTILINE))
            self.assertTrue(re.search('^20.000, 1, swarm_0, 31.756, 649.402, -0.847, 1.471, 0.001, 0.000, 30.000, -0.001, 5.259', csv, re.MULTILINE))
            self.assertTrue(re.search('^20.000, 18, swarm_10, 25.980, 586.474, -0.830, 1.488, 6.283, 0.000, 30.250, -0.001, 4.558', csv, re.MULTILINE))
        elif platform == "linux" or platform == "linux2":
            self.assertTrue(re.search('^5.000, 0, Ego, 11.090, 349.861, -0.625, 1.550, 0.002, 0.000, 10.000, -0.000, 4.627', csv, re.MULTILINE))
            self.assertTrue(re.search('^5.000, 1, swarm_0, 9.030, 200.000, -0.348, 1.562, 0.002, 0.000, 30.000, -0.000, 1.315', csv, re.MULTILINE))
            self.assertTrue(re.search('^5.000, 16, swarm_14, 8.516, 126.491, -0.197, 1.565, 0.002, 0.000, 30.542, -0.000, 1.759', csv, re.MULTILINE))
            self.assertTrue(re.search('^5.000, 17, swarm_14\\+, 8.484, 120.491, -0.183, 1.565, 0.002, 0.000, 30.000, -0.000, 1.690', csv, re.MULTILINE))
            self.assertTrue(re.search('^5.000, 18, swarm_14\\+\\+, 8.453, 114.492, -0.169, 1.566, 0.002, 6.283, 30.000, -0.000, 1.690', csv, re.MULTILINE))
            self.assertTrue(re.search('^5.000, 19, swarm_14\\+\\+\\+, 8.419, 107.792, -0.154, 1.566, 0.002, 6.283, 30.000, -0.000, 1.690', csv, re.MULTILINE))
            self.assertTrue(re.search('^20.000, 0, Ego, 16.330, 499.764, -0.841, 1.517, 0.000, 0.000, 10.000, -0.001, 5.942', csv, re.MULTILINE))
            self.assertTrue(re.search('^20.000, 1, swarm_0, 15.199, 477.382, -0.825, 1.524, 0.001, 0.000, 10.156, -0.001, 2.396', csv, re.MULTILINE))
            self.assertTrue(re.search('^20.000, 57, swarm_40, 15.965, 398.132, -0.716, 1.542, 0.002, 0.000, 31.167, -0.001, 3.170', csv, re.MULTILINE))
            self.assertTrue(re.search('^20.000, 58, swarm_40\\+, 15.804, 392.134, -0.705, 1.544, 0.002, 0.000, 30.000, -0.001, 3.015', csv, re.MULTILINE))

    def test_conflicting_domains(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'EnvironmentSimulator/Unittest/xosc/conflicting-domains.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*conflicting-domains.xosc', log))
        self.assertTrue(re.search('^0.00.*Ego New position:.*$\\n^.*Pos\\(20.00, -1.53, 0.00\\) Rot\\(0.00, 0.00, 0.00\\) roadId 0 laneId -1 offset 0.00 t -1.53', log, re.MULTILINE))
        self.assertTrue(re.search('^0.00.*Init Ego TeleportAction initState -> startTransition -> runningState', log, re.MULTILINE))
        self.assertTrue(re.search('^0.00.*Init Ego LongitudinalAction initState -> startTransition -> runningState', log, re.MULTILINE))
        self.assertTrue(re.search('^0.00.*TeleportAction runningState -> endTransition -> completeState', log, re.MULTILINE))

        # Check some scenario events
        self.assertTrue(re.search('^2.00.*: Stopping Init Ego LongitudinalAction on conflicting longitudinal domain\\(s\\)', log, re.MULTILINE))
        self.assertTrue(re.search('^2.00.*: Speed action 2 initState -> startTransition -> runningState', log, re.MULTILINE))
        self.assertTrue(re.search('^2.00.*: Lane offset action 1 initState -> startTransition -> runningState', log, re.MULTILINE))

        self.assertTrue(re.search('^4.01.*: Stopping object Ego Lane offset action 1 on conflicting lateral domain\\(s\\)', log, re.MULTILINE))
        self.assertTrue(re.search('^4.01.*: Lane offset action 1 runningState -> endTransition -> completeState', log, re.MULTILINE))
        self.assertTrue(re.search('^4.01.*: Lane offset action 2 initState -> startTransition -> runningState', log, re.MULTILINE))

        self.assertTrue(re.search('^5.530: Lane offset action 2 runningState -> endTransition -> completeState', log, re.MULTILINE))
        self.assertTrue(re.search('^8.990.*: Speed action 2 runningState -> endTransition -> completeState', log, re.MULTILINE))
        self.assertTrue(re.search('^13.00.*: Stop condition timer expired at 4.00 seconds', log, re.MULTILINE))
        self.assertTrue(re.search('^13.00.*: Stop condition == true, speed: 20.00 >= 20.00, edge: none', log, re.MULTILINE))

        # Check vehicle key positions
        csv = generate_csv()

        self.assertTrue(re.search('^4.010, 0, Ego, 35.969, -0.600, 0.000, 0.109, 0.000, 0.000, 8.000, 0.001, 1.644', csv, re.MULTILINE))
        self.assertTrue(re.search('^4.020, 0, Ego, 36.049, -0.600, 0.000, 0.000, 0.000, 0.000, 8.026, -0.004, 1.873', csv, re.MULTILINE))
        self.assertTrue(re.search('^4.030, 0, Ego, 36.130, -0.600, 0.000, 6.281, 0.000, 0.000, 8.053, -0.009, 2.103', csv, re.MULTILINE))
        self.assertTrue(re.search('^5.550, 0, Ego, 51.786, -1.535, 0.000, 0.000, 0.000, 0.000, 12.661, 0.030, 2.853', csv, re.MULTILINE))

    def test_follow_ghost(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/follow_ghost.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*follow_ghost.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('^3.050.* SpeedEvent1 complete after 1 execution', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^3.510: LaneChangeCondition1 == true, 3.5100 > 3.50 edge: rising', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^19.510: StopCondition == true, element: StopEvent state: COMPLETE, edge: none', log, re.MULTILINE)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^-0.500, 1, Ego_ghost, 8.212, 60.220, -0.057, 1.567, 0.002, 0.000, 10.000, -0.000, 4.153', csv, re.MULTILINE))
        self.assertTrue(re.search('^2.060, 1, Ego_ghost, 8.392, 102.318, -0.142, 1.566, 0.002, 0.000, 22.800, -0.000, 5.052', csv, re.MULTILINE))
        self.assertTrue(re.search('^2.550, 0, Ego, 8.279, 77.039, -0.088, 1.567, 0.002, 0.000, 18.387, -0.000, 1.941', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.500, 0, Ego, 6.152, 178.693, -0.308, 1.637, 0.002, 0.000, 27.778, -0.014, 3.586', csv, re.MULTILINE))
        self.assertTrue(re.search('^13.000, 1, Ego_ghost, 11.260, 357.856, -0.641, 1.549, 0.002, 0.000, 5.100, -0.000, 0.935', csv, re.MULTILINE))
        self.assertTrue(re.search('^13.350, 0, Ego, 10.931, 342.076, -0.610, 1.551, 0.002, 0.000, 10.006, -0.001, 5.970', csv, re.MULTILINE))

    def test_heading_trig(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'EnvironmentSimulator/Unittest/xosc/traj-heading-trig.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*traj-heading-trig.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('^0.010: MyLaneChangeEvent standbyState -> startTransition -> runningState', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^1.300: MyAlignOrientationStartCondition == true, distance 0.48 < tolerance \\(1.50\\), orientation \\[0.05, 0.00, 0.00\\] \\(tolerance 0.05\\), edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^12.690: MyStopCondition == true, distance 0.48 < tolerance \\(1.00\\), orientation \\[0.95, 0.00, 0.00\\] \\(tolerance 0.05\\), edge: none', log, re.MULTILINE)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^1.300, 0, Car0, 438.047, -1.058, 0.000, 0.050, 0.000, 0.000, 13.889, 0.007, 1.322', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.310, 0, Car0, 438.047, -1.058, 0.000, 0.000, 0.000, 0.000, 13.889, 0.002, 1.719', csv, re.MULTILINE))
        self.assertTrue(re.search('^12.690, 0, Car0, 582.260, 41.297, 0.000, 0.951, 0.000, 0.000, 13.889, 0.031, 0.917', csv, re.MULTILINE))

    def test_relative_speed_trig(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'EnvironmentSimulator/Unittest/xosc/direction_dimension_trig.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*direction_dimension_trig.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('^2.680: EgoAccelRelCondition == true, relative_speed: -8.00 > -8.00, edge: rising', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^2.680: EgoAccelRelEvent standbyState -> startTransition -> runningState', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^4.670: EgoAccelRelEvent runningState -> endTransition -> completeState', log, re.MULTILINE)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^0.030, 1, OverTaker, 90.001, 19.999, 0.000, 5.356, 0.000, 0.000, 0.042, 0.000, 0.002', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.040, 0, Ego, 50.399, -1.968, 0.000, 0.080, 0.000, 0.000, 10.000, 0.000, 1.143', csv, re.MULTILINE))
        self.assertTrue(re.search('^2.690, 0, Ego, 76.814, 0.145, 0.000, 0.080, 0.000, 0.000, 10.050, 0.000, 1.460', csv, re.MULTILINE))
        self.assertTrue(re.search('^2.690, 1, OverTaker, 93.050, 15.933, 0.000, 5.356, 0.000, 0.000, 3.766, 0.000, 1.960', csv, re.MULTILINE))
        self.assertTrue(re.search('^4.210, 0, Ego, 97.761, 1.821, 0.000, 0.080, 0.000, 0.000, 17.650, 0.000, 5.169', csv, re.MULTILINE))
        self.assertTrue(re.search('^4.210, 1, OverTaker, 97.462, 10.051, 0.000, 5.356, 0.000, 0.000, 5.894, 0.000, 4.116', csv, re.MULTILINE))

    def test_lane_change_at_hw_exit(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'EnvironmentSimulator/Unittest/xosc/highway_exit.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*highway_exit.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('^3.380: position trigger == true, distance 1.70 < tolerance \\(2.00\\), edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^3.380: slowdown initState -> startTransition -> runningState', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^4.380: position trigger == true, distance 1.70 < tolerance \\(2.00\\), edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^4.380: Event slowdown event ended, overwritten by event lanechange event', log, re.MULTILINE)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^3.380, 0, Ego, 128.300, -4.500, 0.000, 0.000, 0.000, 0.000, 35.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^3.380, 1, Target, 148.300, -4.500, 0.000, 0.000, 0.000, 0.000, 35.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^4.390, 0, Ego, 163.650, -4.500, 0.000, 0.000, 0.000, 0.000, 35.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^4.390, 1, Target, 182.296, -4.500, 0.000, 6.283, 0.000, 0.000, 31.250, -0.003, 1.592', csv, re.MULTILINE))
        self.assertTrue(re.search('^7.000, 0, Ego, 255.000, -4.500, 0.000, 0.000, 0.000, 0.000, 35.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^7.000, 1, Target, 263.824, -6.702, 0.000, 6.250, 0.000, 0.000, 31.250, 0.001, 2.150', csv, re.MULTILINE))
        self.assertTrue(re.search('^11.500, 0, Ego, 412.500, -4.500, 0.000, 0.000, 0.000, 0.000, 35.000, 0.000, 0.177', csv, re.MULTILINE))
        self.assertTrue(re.search('^11.500, 1, Target, 400.801, -30.940, 0.000, 5.933, 0.000, 0.000, 31.250, 0.000, 1.812', csv, re.MULTILINE))

    def test_lane_change_clothoid(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/lane-change_clothoid_based_trajectory.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*lane-change_clothoid_based_trajectory.xosc', log)  is not None)
        self.assertTrue(re.search('^Adding clothoid\\(x=0.00 y=0.00 h=0.00 curv=0.01 curvDot=0.00 len=15.00 startTime=0.00 stopTime=0.00\\)', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^Adding clothoid\\(x=0.00 y=0.00 h=0.00 curv=0.00 curvDot=0.00 len=5.00 startTime=0.00 stopTime=0.00\\)', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^Adding clothoid\\(x=0.00 y=0.00 h=0.00 curv=-0.01 curvDot=0.00 len=15.00 startTime=0.00 stopTime=0.00\\)', log, re.MULTILINE)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('^3.090: LaneChangeCondition2 == true, element: LaneChangeEvent1 state: COMPLETE, edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^3.450: LaneChangeCondition3 == true, element: LaneChangeEvent2 state: COMPLETE, edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^4.520: LaneChangeEvent3 runningState -> endTransition -> completeState', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^7.000: StopCondition timer 1.00s started', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^8.000: StopCondition == false, 7.0000 < 7.00 edge: falling', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^8.000: storyBoard runningState -> stopTransition -> completeState', log, re.MULTILINE)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^1.980, 0, Car, 77.500, -1.535, 0.000, 0.000, 0.000, 0.000, 13.889', csv, re.MULTILINE))
        self.assertTrue(re.search('^2.020, 0, Car, 78.056, -1.535, 0.000, 0.001, 0.000, 0.000, 13.889, 0.005, 4.761', csv, re.MULTILINE))
        self.assertTrue(re.search('^3.200, 0, Car, 94.371, -0.184, 0.000, 0.150, 0.000, 0.000, 13.889, 0.000, 1.320', csv, re.MULTILINE))
        self.assertTrue(re.search('^4.150, 0, Car, 107.473, 1.319, 0.000, 0.053, 0.000, 0.000, 13.889, -0.031, 1.320', csv, re.MULTILINE))
        self.assertTrue(re.search('^4.520, 0, Car, 112.609, 1.458, 0.000, 0.001, 0.000, 0.000, 13.889, -0.031, 3.436', csv, re.MULTILINE))

    def test_action_dynamics(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'EnvironmentSimulator/Unittest/xosc/test_action_dynamics.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*test_action_dynamics.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('2.010: LaneChange1Condition == true, 2.0100 > 2.00 edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('16.250: LaneChange2Event complete after 1 execution', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('21.510: LaneChange3Event complete after 1 execution', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('23.010: LaneOffset1Condition == true, 23.0100 > 23.00 edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('28.960: QuitCondition == true, element: LaneOffset1Event state: END_TRANSITION, edge: rising', log, re.MULTILINE)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^2.010, 0, Car, 30.100, 1.535, 0.000, 6.283, 0.000, 0.000, 10.000, 0.000, 0.880', csv, re.MULTILINE))
        self.assertTrue(re.search('^2.020, 0, Car, 30.200, 1.527, 0.000, 6.202, 0.000, 0.000, 10.000, -0.005, 1.166', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.370, 0, Car, 65.070, -2.014, 0.000, 6.121, 0.000, 0.000, 4.960, -0.030, 0.656', csv, re.MULTILINE))
        self.assertTrue(re.search('^7.000, 0, Car, 67.854, -2.527, 0.000, 6.084, 0.000, 0.000, 4.015, -0.056, 2.437', csv, re.MULTILINE))
        self.assertTrue(re.search('^7.010, 0, Car, 67.894, -2.535, 0.000, 0.000, 0.000, 0.000, 4.000, -0.051, 2.551', csv, re.MULTILINE))
        self.assertTrue(re.search('^14.400, 0, Car, 97.242, 0.183, 0.000, 0.154, 0.000, 0.000, 4.000, 0.000, 5.327', csv, re.MULTILINE))
        self.assertTrue(re.search('^16.700, 0, Car, 106.352, 1.344, 0.000, 6.283, 0.000, 0.000, 4.000, 0.000, 0.197', csv, re.MULTILINE))
        self.assertTrue(re.search('^17.820, 0, Car, 110.822, 1.121, 0.000, 6.150, 0.000, 0.000, 4.000, -0.114, 0.431', csv, re.MULTILINE))
        self.assertTrue(re.search('^21.410, 0, Car, 130.653, -1.524, 0.000, 6.272, 0.000, 0.000, 7.000, 0.045, 1.239', csv, re.MULTILINE))
        self.assertTrue(re.search('^22.540, 0, Car, 138.563, -1.529, 0.000, 6.283, 0.000, 0.000, 7.000, 0.000, 4.990', csv, re.MULTILINE))
        self.assertTrue(re.search('^24.800, 0, Car, 154.383, 0.790, 0.000, 0.247, 0.000, 0.000, 7.000, -0.039, 6.207', csv, re.MULTILINE))
        self.assertTrue(re.search('^26.000, 0, Car, 162.783, 1.968, 0.000, 6.283, 0.000, 0.000, 7.000, -0.103, 5.074', csv, re.MULTILINE))

    def test_route_lane_change(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'EnvironmentSimulator/Unittest/xosc/route_lane_change.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*route_lane_change.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('Route::AddWaypoint Added waypoint 0: 2, -1, 100.00', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('Route::AddWaypoint Added intermediate waypoint 1 roadId 15 laneId -1', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('Route::AddWaypoint Added waypoint 2: 1, -1, 5.00', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('0.000: Pos\\(22.11, 6.78, 0.00\\) Rot\\(4.89, 0.00, 0.00\\) roadId 2 laneId -1 offset 0.00 t -1.75', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('31.260: LaneChangeCondition6 == true, element: LaneChangeEvent5 state: END_TRANSITION, edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('38.260: QuitCondition == true, element: LaneChangeEvent6 state: END_TRANSITION, edge: rising', log, re.MULTILINE)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^1.000, 0, Car, 27.199, -1.417, 0.000, 5.799, 0.000, 0.000, 10.000, 0.356, 3.439', csv, re.MULTILINE))
        self.assertTrue(re.search('^3.310, 0, Car, 49.593, 1.863, 0.000, 0.243, 0.000, 0.000, 10.000, -0.000, 0.324', csv, re.MULTILINE))
        self.assertTrue(re.search('^3.320, 0, Car, 49.690, 1.887, 0.000, 0.243, 0.000, 0.000, 10.000, 0.000, 0.609', csv, re.MULTILINE))
        self.assertTrue(re.search('^3.330, 0, Car, 49.752, 1.904, 0.000, 0.243, 0.000, 0.000, 0.000, 0.000, 0.609', csv, re.MULTILINE))
        self.assertTrue(re.search('^7.000, 0, Car, 27.484, 0.507, 0.000, 5.745, 0.000, 0.000, 10.000, 0.276, 4.357', csv, re.MULTILINE))
        self.assertTrue(re.search('^8.600, 0, Car, 42.942, 2.380, 0.000, 0.225, 0.000, 0.000, 10.000, -0.022, 6.089', csv, re.MULTILINE))
        self.assertTrue(re.search('^14.500, 0, Car, 20.349, -4.156, 0.000, 3.655, 0.000, 0.000, 10.000, -0.379, 5.014', csv, re.MULTILINE))
        self.assertTrue(re.search('^16.500, 0, Car, 0.799, -8.131, 0.000, 3.287, 0.000, 0.000, 10.000, 0.000, 5.608', csv, re.MULTILINE))
        self.assertTrue(re.search('^19.200, 0, Car, 29.924, -28.426, 0.000, 1.919, 0.000, 0.000, 10.000, -0.058, 1.070', csv, re.MULTILINE))
        self.assertTrue(re.search('^23.000, 0, Car, 21.760, 8.661, 0.000, 1.753, 0.000, 0.000, 10.000, -0.000, 2.827', csv, re.MULTILINE))
        self.assertTrue(re.search('^27.510, 0, Car, 16.240, -3.138, 0.000, 3.203, 0.000, 0.000, 10.000, -0.379, 6.020', csv, re.MULTILINE))
        self.assertTrue(re.search('^35.050, 0, Car, 23.768, 1.326, 0.000, 5.235, 0.000, 0.000, 10.000, 0.200, 1.537', csv, re.MULTILINE))
        self.assertTrue(re.search('^37.000, 0, Car, 41.512, -1.397, 0.000, 0.193, 0.000, 0.000, 10.000, 0.000, 0.703', csv, re.MULTILINE))

    def test_drop_bike(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/drop-bike.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*drop-bike.xosc', log)  is not None)

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
        self.assertTrue(re.search('Loading .*speed_over_distance.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('^10.080: SpeedChangeCondition2 == true, distance 0.93 < tolerance \\(1.00\\), edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^26.880: SpeedChangeCondition3 == true, distance 0.99 < tolerance \\(1.00\\), edge: none', log, re.MULTILINE)  is not None)

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

        # Explicit collision detection in condition when global collision detection is disabled

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*test-collision-detection.xosc', log)  is not None)

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

        # Same as previous, but making use of enabled global collision detection in condition

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*test-collision-detection.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('^5.250: Collision between Ego and NPC2', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^5.250: collision 0 between Ego and NPC2', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^6.260: Collision between Ego and NPC1', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^6.260: collision 0 between Ego and NPC1', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^7.100: Collision between Ego and NPC2 dissolved', log, re.MULTILINE)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^20.000, 0, Ego, 110.000, -0.035, 0.000, 0.000, 0.000, 0.000, 5.000, 0.000, 2.971', csv, re.MULTILINE))
        self.assertTrue(re.search('^20.000, 1, NPC1, 60.000, -1.535, 0.000, 0.000, 0.000, 0.000, 1.000, 0.000, 0.594', csv, re.MULTILINE))
        self.assertTrue(re.search('^20.000, 2, NPC2, 30.000, 1.535, 0.000, 3.142, 0.000, 0.000, 1.000, 0.000, 0.594', csv, re.MULTILINE))

    def test_add_delete_entity(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'EnvironmentSimulator/Unittest/xosc/add_delete_entity.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*add_delete_entity.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('^2.000: Failed to activate obj Car1. Already active \\(1 instances in active list\\)', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^3.010: Added entity Car2', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^7.010: Deleted entity Car2', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^9.010: Failed to deactivate obj Car2. Already inactive \\(0 in active list\\)', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^10.010: Deleted entity Car1', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^12.010: Added entity Car1', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^15.900: Deleted entity Car1', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^17.910: StopCondition timer expired at 2.00 seconds', log, re.MULTILINE)  is not None)

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
        self.assertTrue(re.search('Loading .*multi_lane_changes.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('^0.010: act_start == true, 0.0100 > 0.00 edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^1.140: start_trigger1 == true, rel_dist: 0.08 < 0.10, edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^3.350: start_trigger2 == true, rel_dist: 0.08 < 0.10, edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^7.350: start_trigger3 timer expired at 1.00 seconds', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^15.330: start_trigger6 == true, HWT: 1.00 > 1.00, edge none', log, re.MULTILINE)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^5.500, 0, Ego, 109.028, -6.000, 0.000, 0.000, 0.000, 0.000, 15.278', csv, re.MULTILINE))
        self.assertTrue(re.search('^5.560, 1, Target1, 120.051, -6.000, 0.000, 0.000, 0.000, 0.000, 15.278, 0.000, 5.954', csv, re.MULTILINE))
        self.assertTrue(re.search('^5.560, 2, Target2, 129.944, -6.000, 0.000, 0.000, 0.000, 0.000, 15.278, 0.000, 3.937', csv, re.MULTILINE))
        self.assertTrue(re.search('^5.560, 3, Target3, 141.793, -6.000, 0.000, 0.000, 0.000, 0.000, 16.624, 0.000, 5.180', csv, re.MULTILINE))
        self.assertTrue(re.search('^12.200, 0, Ego, 211.389, -6.000, 0.000, 0.000, 0.000, 0.000, 15.278, 0.000, 4.752', csv, re.MULTILINE))
        self.assertTrue(re.search('^12.200, 1, Target1, 247.839, -9.275, 0.000, 0.107, 0.000, 0.000, 22.609, 0.021, 1.403', csv, re.MULTILINE))
        self.assertTrue(re.search('^12.200, 2, Target2, 234.910, -2.055, 0.000, 0.035, 0.000, 0.000, 20.711, -0.036, 3.218', csv, re.MULTILINE))
        self.assertTrue(re.search('^12.200, 3, Target3, 243.594, -6.000, 0.000, 0.000, 0.000, 0.000, 15.278, 0.000, 0.729', csv, re.MULTILINE))
        self.assertTrue(re.search('^20.000, 0, Ego, 330.556, -6.000, 0.000, 0.000, 0.000, 0.000, 15.278, 0.000, 5.936', csv, re.MULTILINE))
        self.assertTrue(re.search('^20.000, 1, Target1, 374.460, -6.000, 0.000, 0.000, 0.000, 0.000, 15.278, 0.000, 5.479', csv, re.MULTILINE))
        self.assertTrue(re.search('^20.000, 2, Target2, 389.874, -6.000, 0.000, 0.000, 0.000, 0.000, 15.278, 0.000, 0.384', csv, re.MULTILINE))
        self.assertTrue(re.search('^20.000, 3, Target3, 362.761, -6.000, 0.000, 0.000, 0.000, 0.000, 15.278, 0.000, 1.913', csv, re.MULTILINE))

    def test_init_cases(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'EnvironmentSimulator/Unittest/xosc/init_test.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*init_test.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('^-2.000: Ego New position:\\n-2.000: Pos\\(200.00, -1.53, 0.00\\) '\
            'Rot\\(0.00, 0.00, 0.00\\) roadId 1 laneId -1 offset 0.00 t -1.53', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^-2.000: OverTaker2 New position:\\n-2.000: Pos\\(280.00, -1.53, 0.00\\) '\
            'Rot\\(0.00, 0.00, 0.00\\) roadId 1 laneId -1 offset 0.00 t -1.53', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^-2.000: Ego_ghost New position:\\n-2.000: Pos\\(200.00, -1.53, 0.00\\) '\
            'Rot\\(0.00, 0.00, 0.00\\) roadId 1 laneId -1 offset 0.00 t -1.53', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^-2.000: Init OverTaker1 LongitudinalAction runningState -> endTransition -> completeState', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^0.990: Init OverTaker1 RoutingAction runningState -> endTransition -> completeState', log, re.MULTILINE)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^-2.000, 0, Ego, 200.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^-2.000, 1, OverTaker1, 250.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^-2.000, 2, OverTaker2, 280.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^-2.000, 3, Ego_ghost, 200.000, -1.535, 0.000, 0.102, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^-1.950, 0, Ego, 200.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^-1.950, 1, OverTaker1, 250.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^-1.950, 2, OverTaker2, 280.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^-1.950, 3, Ego_ghost, 200.497, -1.484, 0.000, 0.102, 0.000, 0.000, 10.000, 0.000, 1.429', csv, re.MULTILINE))
        self.assertTrue(re.search('^-1.900, 0, Ego, 200.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^-1.900, 1, OverTaker1, 250.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^-1.900, 2, OverTaker2, 280.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^-1.900, 3, Ego_ghost, 200.995, -1.433, 0.000, 0.102, 0.000, 0.000, 10.000, 0.000, 2.857', csv, re.MULTILINE))

        self.assertTrue(re.search('^-0.500, 0, Ego, 200.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^-0.500, 1, OverTaker1, 250.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^-0.500, 2, OverTaker2, 280.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^-0.500, 3, Ego_ghost, 214.921, 0.000, 0.000, 0.102, 0.000, 0.000, 10.000, 0.000, 5.158', csv, re.MULTILINE))

        self.assertTrue(re.search('^0.000, 0, Ego, 200.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 1, OverTaker1, 250.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 2, OverTaker2, 280.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 3, Ego_ghost, 219.895, 0.512, 0.000, 0.102, 0.000, 0.000, 10.000, 0.000, 0.594', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.010, 0, Ego, 200.100, -1.533, 0.000, 0.002, 0.000, 0.000, 10.000, 0.103, 0.286', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.010, 1, OverTaker1, 250.100, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.286', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.010, 2, OverTaker2, 280.100, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.286', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.010, 3, Ego_ghost, 219.994, 0.522, 0.000, 0.102, 0.000, 0.000, 10.000, 0.000, 0.880', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.020, 0, Ego, 200.200, -1.532, 0.000, 0.004, 0.000, 0.000, 10.000, 0.101, 0.571', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.020, 1, OverTaker1, 250.200, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.571', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.020, 2, OverTaker2, 280.200, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.571', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.020, 3, Ego_ghost, 220.094, 0.532, 0.000, 0.102, 0.000, 0.000, 10.000, 0.000, 1.166', csv, re.MULTILINE))

    def test_reverse_lane_change(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'EnvironmentSimulator/Unittest/xosc/reverse_lane_change.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*reverse_lane_change.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('^1.010: reverse_trigger == true, 1.0100 > 1.00 edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^5.010: lateral_event_trigger == true, 5.0100 > 5.00 edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^5.370: reverse_event complete after 1 execution', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^8.000: lateral_event complete after 1 execution', log, re.MULTILINE)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^2.000, 0, Ego, 276.626, -1.535, 0.000, 0.000, 0.000, 0.000, 10.507, 0.000, 0.675', csv, re.MULTILINE))
        self.assertTrue(re.search('^3.190, 0, Ego, 283.248, -1.535, 0.000, 0.000, 0.000, 0.000, 0.017, 0.000, 0.748', csv, re.MULTILINE))
        self.assertTrue(re.search('^3.200, 0, Ego, 283.248, -1.535, 0.000, 0.000, 0.000, 0.000, -0.083, 0.000, 0.745', csv, re.MULTILINE))
        self.assertTrue(re.search('^7.000, 0, Ego, 241.403, 0.725, 0.000, 6.185, 0.000, 0.000, -13.889, -0.011, -6.019', csv, re.MULTILINE))
        self.assertTrue(re.search('^10.000, 0, Ego, 199.766, 1.535, 0.000, 0.000, 0.000, 0.000, -13.889, 0.000, -5.686', csv, re.MULTILINE))

    def test_ghost_restart(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'EnvironmentSimulator/Unittest/xosc/ghost_restart.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*ghost_restart.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('^-0.500: Pos\\(50.00, -11.50, 0.00\\) Rot\\(0.00, 0.00, 0.00\\) roadId 1 laneId -4 offset 0.00 t -11.50', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^1.000: Set parameter myparam = true', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^1.010: parameter myparam 0 == true edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^1.010: Pos\\(60.10, -11.50, 0.00\\) Rot\\(0.00, 0.00, 0.00\\) roadId 1 laneId -4 offset 0.00 t -11.50', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^1.010: AddedGhostTeleport runningState -> endTransition -> completeState', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^2.500: newspeed runningState -> endTransition -> completeState', log, re.MULTILINE)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^-0.500, 0, Ego, 50.000, -11.500, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^-0.500, 1, Ego_ghost, 50.000, -11.500, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.010, 1, Ego_ghost, 65.100, -11.500, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 5.444', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.020, 1, Ego_ghost, 65.121, -10.608, 0.000, 0.173, 0.000, 0.000, 10.000, 0.000, 0.880', csv, re.MULTILINE))
        self.assertTrue(re.search('^2.000, 0, Ego, 69.847, -9.934, 0.000, 0.221, 0.000, 0.000, 10.000, 0.009, 0.566', csv, re.MULTILINE))
        self.assertTrue(re.search('^2.000, 1, Ego_ghost, 74.770, -8.892, 0.000, 0.173, 0.000, 0.000, 10.000, 0.000, 3.747', csv, re.MULTILINE))
        self.assertTrue(re.search('^5.000, 0, Ego, 99.649, -8.042, 0.000, 6.283, 0.000, 0.000, 10.000, 0.014, 4.598', csv, re.MULTILINE))
        self.assertTrue(re.search('^5.000, 1, Ego_ghost, 104.691, -8.000, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 1.497', csv, re.MULTILINE))

    def test_ghost_restart2(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'EnvironmentSimulator/Unittest/xosc/ghost_restart2.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*ghost_restart2.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('^-2.000: Car New position:\\n-2.000: Pos\\(20.00, -1.53, 0.00\\) Rot\\(0.00, 0.00, 0.00\\) roadId 1 laneId -1 offset 0.00 t -1.53', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^3.000: Set parameter myparam = 1', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^3.010: parameter myparam 1 == 1 edge: rising', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^3.010: Car_ghost New position:\\n3.010: Pos\\(50.10, -1.53, 0.00\\) Rot\\(0.00, 0.00, 0.00\\) roadId 1 laneId -1 offset 0.00 t -1.53', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^3.010: AddedGhostTeleport runningState -> endTransition -> completeState', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^5.010: BrakeAction1 runningState -> endTransition -> completeState', log, re.MULTILINE)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^-2.000, 0, Car, 20.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^-2.000, 1, Car2, 20.000, 1.535, 0.000, 6.283, 0.000, 0.000, 15.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^-2.000, 2, Car_ghost, 20.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))

        self.assertTrue(re.search('^-0.050, 0, Car, 20.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^-0.050, 1, Car2, 20.000, 1.535, 0.000, 6.283, 0.000, 0.000, 15.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^-0.050, 2, Car_ghost, 39.500, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 5.449', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 0, Car, 20.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 1, Car2, 20.000, 1.535, 0.000, 6.283, 0.000, 0.000, 15.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 2, Car_ghost, 40.000, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.594', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.010, 0, Car, 20.100, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.286', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.010, 1, Car2, 20.150, 1.535, 0.000, 6.283, 0.000, 0.000, 14.979, 0.000, 0.428', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.010, 2, Car_ghost, 40.100, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.880', csv, re.MULTILINE))
        self.assertTrue(re.search('^3.010, 0, Car, 50.100, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, -?0.000, 4.319', csv, re.MULTILINE))
        self.assertTrue(re.search('^3.010, 1, Car2, 55.410, 1.535, 0.000, 6.283, 0.000, 0.000, 8.550, -?0.000, 0.642', csv, re.MULTILINE))
        self.assertTrue(re.search('^3.010, 2, Car_ghost, 70.100, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, -?0.000, 4.913', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.070, 0, Car, 50.199, -1.535, 0.000, 0.001, 0.000, 0.000, 9.900, 0.031, 4.601', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.070, 1, Car2, 55.496, 1.535, 0.000, 6.283, 0.000, 0.000, 8.529, 0.000, 0.886', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.070, 2, Car_ghost, 50.596, -1.535, 0.000, 0.000, 0.000, 0.000, 9.914, 0.000, 0.046', csv, re.MULTILINE))
        self.assertTrue(re.search('^5.010, 0, Car, 65.363, -1.534, 0.000, 6.283, 0.000, 0.000, 6.135, 0.000, 3.945', csv, re.MULTILINE))
        self.assertTrue(re.search('^5.010, 1, Car2, 68.203, 1.535, 0.000, 6.283, 0.000, 0.000, 4.264, 0.000, 5.777', csv, re.MULTILINE))
        self.assertTrue(re.search('^5.010, 2, Car_ghost, 78.486, -1.535, 0.000, 0.000, 0.000, 0.000, 4.286, 0.000, 4.334', csv, re.MULTILINE))
        self.assertTrue(re.search('^3.070, 0, Car, 65.424, -1.534, 0.000, 0.000, 0.000, 0.000, 6.035, 0.023, 4.118', csv, re.MULTILINE))
        self.assertTrue(re.search('^3.070, 1, Car2, 68.246, 1.535, 0.000, 6.283, 0.000, 0.000, 4.243, 0.000, 5.898', csv, re.MULTILINE))
        self.assertTrue(re.search('^3.070, 2, Car_ghost, 65.661, -1.534, 0.000, 6.283, 0.000, 0.000, 5.951, -0.001, 5.184', csv, re.MULTILINE))
        self.assertTrue(re.search('^8.010, 0, Car, 67.260, -1.534, 0.000, 0.000, 0.000, 0.000, 0.000, -0.000, 3.081', csv, re.MULTILINE))
        self.assertTrue(re.search('^8.010, 1, Car2, 72.425, 1.535, 0.000, 6.283, 0.000, 0.000, 0.000, 0.000, 5.272', csv, re.MULTILINE))
        self.assertTrue(re.search('^8.010, 2, Car_ghost, 71.285, -1.534, 0.000, 6.283, 0.000, 0.000, 0.000, 0.000, 2.405', csv, re.MULTILINE))

    def test_maneuver_groups_x_3(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'EnvironmentSimulator/Unittest/xosc/maneuver_groups_x_3.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*maneuver_groups_x_3.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('^0.010: MyMG initState -> startTransition -> runningState', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^1.130: MyAction2Trigger == true, element: MyEvent1 state: END_TRANSITION, edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^3.370: MyAction2Trigger == true, element: MyEvent1 state: END_TRANSITION, edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^5.610: MyAction2Trigger == true, element: MyEvent1 state: END_TRANSITION, edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^6.720: MyMG complete after 3 executions', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^8.010: storyBoard runningState -> stopTransition -> completeState', log, re.MULTILINE)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^5.200, 0, Target, 66.000, -1.186, 0.000, 0.129, 0.000, 0.000, 5.000, -0.033, 5.171', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.790, 0, Target, 73.950, -1.535, 0.000, 0.000, 0.000, 0.000, 5.000, 0.208, 2.752', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.800, 0, Target, 74.000, -1.535, 0.000, 0.000, 0.000, 0.000, 5.000, 0.203, 2.895', csv, re.MULTILINE))

    def test_speed_profile(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'EnvironmentSimulator/Unittest/xosc/speed-profile_test.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*speed-profile_test.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('^2.000: EventTrigger1 == true, 2.0000 >= 2.00 edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^2.000: EventTrigger2 == true, 2.0000 >= 2.00 edge: none', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^2.000: SpeedProfileAction1 initState -> startTransition -> runningState', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^23.340: SpeedProfile: Can\'t reach.* speed 5.00 on.* time 26.34s.* extend to 26.76s', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^25.340: SpeedProfile: Can\'t reach.* speed 0.00 on.* time 26.24s.* extend to 27.54s', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^29.550: SpeedProfile: Can\'t reach.* speed 0.00 on.* time 30.05s.* extend to 34.49s', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^37.490: StopSimulationCondition == true, element: SpeedProfileEvent7 state: COMPLETE, edge: rising', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^37.490: storyBoard runningState -> stopTransition -> completeState', log, re.MULTILINE)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('^5.990, 0, Car1, 26.941, -1.535, 0.000, 0.000, 0.000, 0.000, 8.874, 0.000, 4.421', csv, re.MULTILINE))
        self.assertTrue(re.search('^5.990, 1, Car2, 29.950, 1.535, 0.000, 0.000, 0.000, 0.000, 9.975, 0.000, 0.451', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 0, Car1, 27.030, -1.535, 0.000, 0.000, 0.000, 0.000, 8.885, 0.000, 4.674', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 1, Car2, 30.050, 1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.737', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.010, 0, Car1, 27.119, -1.535, 0.000, 0.000, 0.000, 0.000, 8.896, 0.000, 4.929', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.010, 1, Car2, 30.150, 1.535, 0.000, 0.000, 0.000, 0.000, 9.985, 0.000, 1.022', csv, re.MULTILINE))

        self.assertTrue(re.search('^9.990, 0, Car1, 54.699, -1.535, 0.000, 0.000, 0.000, 0.000, 4.381, 0.000, 2.048', csv, re.MULTILINE))
        self.assertTrue(re.search('^9.990, 1, Car2, 57.980, 1.535, 0.000, 0.000, 0.000, 0.000, 4.015, 0.000, 5.139', csv, re.MULTILINE))
        self.assertTrue(re.search('^10.000, 0, Car1, 54.743, -1.535, 0.000, 0.000, 0.000, 0.000, 4.383, 0.000, 2.173', csv, re.MULTILINE))
        self.assertTrue(re.search('^10.000, 1, Car2, 58.020, 1.535, 0.000, 0.000, 0.000, 0.000, 4.000, 0.000, 5.253', csv, re.MULTILINE))
        self.assertTrue(re.search('^10.010, 0, Car1, 54.787, -1.535, 0.000, 0.000, 0.000, 0.000, 4.386, 0.000, 2.298', csv, re.MULTILINE))
        self.assertTrue(re.search('^10.010, 1, Car2, 58.060, 1.535, 0.000, 0.000, 0.000, 0.000, 4.020, 0.000, 5.368', csv, re.MULTILINE))

        self.assertTrue(re.search('^15.500, 0, Car1, 94.783, -1.535, 0.000, 0.000, 0.000, 0.000, 8.051, 0.000, 3.475', csv, re.MULTILINE))
        self.assertTrue(re.search('^15.500, 1, Car2, 98.040, 1.535, 0.000, 0.000, 0.000, 0.000, 8.000, 0.000, 0.215', csv, re.MULTILINE))
        self.assertTrue(re.search('^19.500, 0, Car1, 140.675, -1.535, 0.000, 0.000, 0.000, 0.000, 14.647, 0.000, 2.649', csv, re.MULTILINE))
        self.assertTrue(re.search('^20.300, 0, Car1, 152.578, -1.535, 0.000, 0.000, 0.000, 0.000, 14.999, 0.000, 5.242', csv, re.MULTILINE))
        self.assertTrue(re.search('^20.310, 0, Car1, 152.728, -1.535, 0.000, 0.000, 0.000, 0.000, 15.000, 0.000, 5.670', csv, re.MULTILINE))
        self.assertTrue(re.search('^25.000, 0, Car1, 220.770, -1.535, 0.000, 0.000, 0.000, 0.000, 10.867, 0.000, 5.298', csv, re.MULTILINE))
        self.assertTrue(re.search('^28.550, 0, Car1, 231.128, -1.535, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 3.476', csv, re.MULTILINE))
        self.assertTrue(re.search('^31.210, 0, Car1, 246.576, -1.535, 0.000, 0.000, 0.000, 0.000, 9.167, 0.000, 3.630', csv, re.MULTILINE))
        self.assertTrue(re.search('^34.470, 0, Car1, 262.293, -1.535, 0.000, 0.000, 0.000, 0.000, 0.001, 0.000, 4.555', csv, re.MULTILINE))
        self.assertTrue(re.search('^34.480, 0, Car1, 262.293, -1.535, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 4.555', csv, re.MULTILINE))
        self.assertTrue(re.search('^34.480, 1, Car2, 249.880, 1.535, 0.000, 0.000, 0.000, 0.000, 8.000, 0.000, 0.504', csv, re.MULTILINE))

    def test_star(self):
        # star is a synthetic scenario involving permutations of road heading, pitch and relative road position
        log = run_scenario(os.path.join(ESMINI_PATH, 'EnvironmentSimulator/Unittest/xosc/star.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*star.xosc', log)  is not None)

        # Check some scenario events

        self.assertTrue(re.search('^0.000: Ego_0_-1 New position:$\\n^0.000: Pos\\(45.00, -1.75, 0.00\\) Rot\\(0.00, 0.00, 0.00\\) roadId 0 laneId -1 offset 0.00 t -1.75', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^4.990: Init Ego_23_1 LateralAction runningState -> endTransition -> completeState', log, re.MULTILINE)  is not None)

        # Check vehicle key positions
        csv = generate_csv()

        self.assertTrue(re.search('^0.000, 0, Ego_0_-1, 45.000, -1.750, 0.000, 2.805, 0.000, 0.000, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 1, Ego_0_1, 45.000, 1.750, 0.000, 3.478, 0.000, 0.000, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 2, Ego_1_-1, 45.000, -1.750, -21.459, 2.752, 5.767, 0.200, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 3, Ego_1_1, 45.000, 1.750, -21.459, 3.531, 5.767, 6.083, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 4, Ego_2_-1, 45.000, -1.750, 21.459, 2.752, 0.516, 6.083, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 5, Ego_2_1, 45.000, 1.750, 21.459, 3.531, 0.516, 0.200, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 6, Ego_3_-1, 33.057, 30.582, 0.000, 3.590, 0.000, 0.000, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 7, Ego_3_1, 30.582, 33.057, 0.000, 4.264, 0.000, 0.000, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 8, Ego_4_-1, 33.057, 30.582, -21.459, 3.537, 5.767, 0.200, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 9, Ego_4_1, 30.582, 33.057, -21.459, 4.317, 5.767, 6.083, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 10, Ego_5_-1, 33.057, 30.582, 21.459, 3.537, 0.516, 6.083, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 11, Ego_5_1, 30.582, 33.057, 21.459, 4.317, 0.516, 0.200, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 12, Ego_6_-1, 1.750, 45.000, 0.000, 4.376, 0.000, 0.000, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 13, Ego_6_1, -1.750, 45.000, 0.000, 5.049, 0.000, 0.000, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 14, Ego_7_-1, 1.750, 45.000, -21.459, 4.323, 5.767, 0.200, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 15, Ego_7_1, -1.750, 45.000, -21.459, 5.102, 5.767, 6.083, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 16, Ego_8_-1, 1.750, 45.000, 21.459, 4.323, 0.516, 6.083, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 17, Ego_8_1, -1.750, 45.000, 21.459, 5.102, 0.516, 0.200, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 18, Ego_9_-1, -30.582, 33.057, 0.000, 5.161, 0.000, 0.000, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 19, Ego_9_1, -33.057, 30.582, 0.000, 5.834, 0.000, 0.000, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 20, Ego_10_-1, -30.582, 33.057, -21.459, 5.108, 5.767, 0.200, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 21, Ego_10_1, -33.057, 30.582, -21.459, 5.887, 5.767, 6.083, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 22, Ego_11_-1, -30.582, 33.057, 21.459, 5.108, 0.516, 6.083, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 23, Ego_11_1, -33.057, 30.582, 21.459, 5.887, 0.516, 0.200, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 24, Ego_12_-1, -45.000, 1.750, 0.000, 5.947, 0.000, 0.000, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 25, Ego_12_1, -45.000, -1.750, 0.000, 0.337, 0.000, 0.000, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 26, Ego_13_-1, -45.000, 1.750, -21.459, 5.894, 5.767, 0.200, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 27, Ego_13_1, -45.000, -1.750, -21.459, 0.390, 5.767, 6.083, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 28, Ego_14_-1, -45.000, 1.750, 21.459, 5.894, 0.516, 6.083, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 29, Ego_14_1, -45.000, -1.750, 21.459, 0.390, 0.516, 0.200, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 30, Ego_15_-1, -33.057, -30.582, 0.000, 0.449, 0.000, 0.000, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 31, Ego_15_1, -30.582, -33.057, 0.000, 1.122, 0.000, 0.000, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 32, Ego_16_-1, -33.057, -30.582, -21.459, 0.396, 5.767, 0.200, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 33, Ego_16_1, -30.582, -33.057, -21.459, 1.175, 5.767, 6.083, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 34, Ego_17_-1, -33.057, -30.582, 21.459, 0.396, 0.516, 6.083, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 35, Ego_17_1, -30.582, -33.057, 21.459, 1.175, 0.516, 0.200, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 36, Ego_18_-1, -1.750, -45.000, 0.000, 1.234, 0.000, 0.000, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 37, Ego_18_1, 1.750, -45.000, 0.000, 1.907, 0.000, 0.000, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 38, Ego_19_-1, -1.750, -45.000, -21.459, 1.181, 5.767, 0.200, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 39, Ego_19_1, 1.750, -45.000, -21.459, 1.960, 5.767, 6.083, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 40, Ego_20_-1, -1.750, -45.000, 21.459, 1.181, 0.516, 6.083, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 41, Ego_20_1, 1.750, -45.000, 21.459, 1.960, 0.516, 0.200, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 42, Ego_21_-1, 30.582, -33.057, 0.000, 2.020, 0.000, 0.000, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 43, Ego_21_1, 33.057, -30.582, 0.000, 2.693, 0.000, 0.000, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 44, Ego_22_-1, 30.582, -33.057, -21.459, 1.967, 5.767, 0.200, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 45, Ego_22_1, 33.057, -30.582, -21.459, 2.746, 5.767, 6.083, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 46, Ego_23_-1, 30.582, -33.057, 21.459, 1.967, 0.516, 6.083, 2.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 47, Ego_23_1, 33.057, -30.582, 21.459, 2.746, 0.516, 0.200, 2.000, 0.000, 0.000', csv, re.MULTILINE))

        self.assertTrue(re.search('^1.500, 0, Ego_0_-1, 42.190, -0.700, 0.000, 2.805, 0.000, 0.000, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 1, Ego_0_1, 42.190, 0.700, 0.000, 3.478, 0.000, 0.000, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 2, Ego_1_-1, 42.190, -0.700, -19.736, 2.752, 5.767, 0.200, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 3, Ego_1_1, 42.190, 0.700, -19.736, 3.531, 5.767, 6.083, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 4, Ego_2_-1, 42.190, -0.700, 19.736, 2.752, 0.516, 6.083, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 5, Ego_2_1, 42.190, 0.700, 19.736, 3.531, 0.516, 0.200, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 6, Ego_3_-1, 30.328, 29.338, 0.000, 3.590, 0.000, 0.000, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 7, Ego_3_1, 29.338, 30.328, 0.000, 4.264, 0.000, 0.000, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 8, Ego_4_-1, 30.328, 29.338, -19.736, 3.537, 5.767, 0.200, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 9, Ego_4_1, 29.338, 30.328, -19.736, 4.317, 5.767, 6.083, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 10, Ego_5_-1, 30.328, 29.338, 19.736, 3.537, 0.516, 6.083, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 11, Ego_5_1, 29.338, 30.328, 19.736, 4.317, 0.516, 0.200, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 12, Ego_6_-1, 0.700, 42.190, 0.000, 4.376, 0.000, 0.000, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 13, Ego_6_1, -0.700, 42.190, 0.000, 5.049, 0.000, 0.000, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 14, Ego_7_-1, 0.700, 42.190, -19.736, 4.323, 5.767, 0.200, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 15, Ego_7_1, -0.700, 42.190, -19.736, 5.102, 5.767, 6.083, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 16, Ego_8_-1, 0.700, 42.190, 19.736, 4.323, 0.516, 6.083, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 17, Ego_8_1, -0.700, 42.190, 19.736, 5.102, 0.516, 0.200, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 18, Ego_9_-1, -29.338, 30.328, 0.000, 5.161, 0.000, 0.000, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 19, Ego_9_1, -30.328, 29.338, 0.000, 5.834, 0.000, 0.000, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 20, Ego_10_-1, -29.338, 30.328, -19.736, 5.108, 5.767, 0.200, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 21, Ego_10_1, -30.328, 29.338, -19.736, 5.887, 5.767, 6.083, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 22, Ego_11_-1, -29.338, 30.328, 19.736, 5.108, 0.516, 6.083, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 23, Ego_11_1, -30.328, 29.338, 19.736, 5.887, 0.516, 0.200, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 24, Ego_12_-1, -42.190, 0.700, 0.000, 5.947, 0.000, 0.000, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 25, Ego_12_1, -42.190, -0.700, 0.000, 0.337, 0.000, 0.000, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 26, Ego_13_-1, -42.190, 0.700, -19.736, 5.894, 5.767, 0.200, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 27, Ego_13_1, -42.190, -0.700, -19.736, 0.390, 5.767, 6.083, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 28, Ego_14_-1, -42.190, 0.700, 19.736, 5.894, 0.516, 6.083, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 29, Ego_14_1, -42.190, -0.700, 19.736, 0.390, 0.516, 0.200, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 30, Ego_15_-1, -30.328, -29.338, 0.000, 0.449, 0.000, 0.000, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 31, Ego_15_1, -29.338, -30.328, 0.000, 1.122, 0.000, 0.000, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 32, Ego_16_-1, -30.328, -29.338, -19.736, 0.396, 5.767, 0.200, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 33, Ego_16_1, -29.338, -30.328, -19.736, 1.175, 5.767, 6.083, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 34, Ego_17_-1, -30.328, -29.338, 19.736, 0.396, 0.516, 6.083, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 35, Ego_17_1, -29.338, -30.328, 19.736, 1.175, 0.516, 0.200, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 36, Ego_18_-1, -0.700, -42.190, 0.000, 1.234, 0.000, 0.000, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 37, Ego_18_1, 0.700, -42.190, 0.000, 1.907, 0.000, 0.000, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 38, Ego_19_-1, -0.700, -42.190, -19.736, 1.181, 5.767, 0.200, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 39, Ego_19_1, 0.700, -42.190, -19.736, 1.960, 5.767, 6.083, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 40, Ego_20_-1, -0.700, -42.190, 19.736, 1.181, 0.516, 6.083, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 41, Ego_20_1, 0.700, -42.190, 19.736, 1.960, 0.516, 0.200, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 42, Ego_21_-1, 29.338, -30.328, 0.000, 2.020, 0.000, 0.000, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 43, Ego_21_1, 30.328, -29.338, 0.000, 2.693, 0.000, 0.000, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 44, Ego_22_-1, 29.338, -30.328, -19.736, 1.967, 5.767, 0.200, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 45, Ego_22_1, 30.328, -29.338, -19.736, 2.746, 5.767, 6.083, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 46, Ego_23_-1, 29.338, -30.328, 19.736, 1.967, 0.516, 6.083, 2.000, 0.000, 2.288', csv, re.MULTILINE))
        self.assertTrue(re.search('^1.500, 47, Ego_23_1, 30.328, -29.338, 19.736, 2.746, 0.516, 0.200, 2.000, 0.000, 2.288', csv, re.MULTILINE))

        self.assertTrue(re.search('^6.000, 0, Ego_0_-1, 33.633, 1.750, 0.000, 3.142, 0.000, 0.000, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 1, Ego_0_1, 33.633, -1.750, 0.000, 3.142, 0.000, 0.000, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 2, Ego_1_-1, 33.633, 1.750, -14.489, 3.142, 5.733, 0.000, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 3, Ego_1_1, 33.633, -1.750, -14.489, 3.142, 5.733, 0.000, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 4, Ego_2_-1, 33.633, 1.750, 14.489, 3.142, 0.550, 6.283, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 5, Ego_2_1, 33.633, -1.750, 14.489, 3.142, 0.550, 6.283, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 6, Ego_3_-1, 22.544, 25.019, 0.000, 3.927, 0.000, 0.000, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 7, Ego_3_1, 25.019, 22.544, 0.000, 3.927, 0.000, 0.000, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 8, Ego_4_-1, 22.544, 25.019, -14.489, 3.927, 5.733, 0.000, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 9, Ego_4_1, 25.019, 22.544, -14.489, 3.927, 5.733, 0.000, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 10, Ego_5_-1, 22.544, 25.019, 14.489, 3.927, 0.550, 6.283, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 11, Ego_5_1, 25.019, 22.544, 14.489, 3.927, 0.550, 6.283, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 12, Ego_6_-1, -1.750, 33.633, 0.000, 4.712, 0.000, 0.000, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 13, Ego_6_1, 1.750, 33.633, 0.000, 4.712, 0.000, 0.000, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 14, Ego_7_-1, -1.750, 33.633, -14.489, 4.712, 5.733, 0.000, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 15, Ego_7_1, 1.750, 33.633, -14.489, 4.712, 5.733, 0.000, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 16, Ego_8_-1, -1.750, 33.633, 14.489, 4.712, 0.550, 6.283, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 17, Ego_8_1, 1.750, 33.633, 14.489, 4.712, 0.550, 6.283, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 18, Ego_9_-1, -25.019, 22.544, 0.000, 5.498, 0.000, 0.000, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 19, Ego_9_1, -22.544, 25.019, 0.000, 5.498, 0.000, 0.000, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 20, Ego_10_-1, -25.019, 22.544, -14.489, 5.498, 5.733, 0.000, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 21, Ego_10_1, -22.544, 25.019, -14.489, 5.498, 5.733, 0.000, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 22, Ego_11_-1, -25.019, 22.544, 14.489, 5.498, 0.550, 6.283, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 23, Ego_11_1, -22.544, 25.019, 14.489, 5.498, 0.550, 6.283, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 24, Ego_12_-1, -33.633, -1.750, 0.000, 6.283, 0.000, 0.000, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 25, Ego_12_1, -33.633, 1.750, 0.000, 6.283, 0.000, 0.000, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 26, Ego_13_-1, -33.633, -1.750, -14.489, 6.283, 5.733, 0.000, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 27, Ego_13_1, -33.633, 1.750, -14.489, 6.283, 5.733, 0.000, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 28, Ego_14_-1, -33.633, -1.750, 14.489, 6.283, 0.550, 6.283, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 29, Ego_14_1, -33.633, 1.750, 14.489, 6.283, 0.550, 6.283, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 30, Ego_15_-1, -22.544, -25.019, 0.000, 0.785, 0.000, 0.000, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 31, Ego_15_1, -25.019, -22.544, 0.000, 0.785, 0.000, 0.000, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 32, Ego_16_-1, -22.544, -25.019, -14.489, 0.785, 5.733, 0.000, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 33, Ego_16_1, -25.019, -22.544, -14.489, 0.785, 5.733, 0.000, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 34, Ego_17_-1, -22.544, -25.019, 14.489, 0.785, 0.550, 6.283, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 35, Ego_17_1, -25.019, -22.544, 14.489, 0.785, 0.550, 6.283, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 36, Ego_18_-1, 1.750, -33.633, 0.000, 1.571, 0.000, 0.000, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 37, Ego_18_1, -1.750, -33.633, 0.000, 1.571, 0.000, 0.000, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 38, Ego_19_-1, 1.750, -33.633, -14.489, 1.571, 5.733, 0.000, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 39, Ego_19_1, -1.750, -33.633, -14.489, 1.571, 5.733, 0.000, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 40, Ego_20_-1, 1.750, -33.633, 14.489, 1.571, 0.550, 6.283, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 41, Ego_20_1, -1.750, -33.633, 14.489, 1.571, 0.550, 6.283, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 42, Ego_21_-1, 25.019, -22.544, 0.000, 2.356, 0.000, 0.000, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 43, Ego_21_1, 22.544, -25.019, 0.000, 2.356, 0.000, 0.000, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 44, Ego_22_-1, 25.019, -22.544, -14.489, 2.356, 5.733, 0.000, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 45, Ego_22_1, 22.544, -25.019, -14.489, 2.356, 5.733, 0.000, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 46, Ego_23_-1, 25.019, -22.544, 14.489, 2.356, 0.550, 6.283, 2.000, 0.000, 2.870', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.000, 47, Ego_23_1, 22.544, -25.019, 14.489, 2.356, 0.550, 6.283, 2.000, 0.000, 2.870', csv, re.MULTILINE))

    def test_star_position_types(self):
        # star_position_types is a synthetic scenario involving further variants of position types
        log = run_scenario(os.path.join(ESMINI_PATH, 'EnvironmentSimulator/Unittest/xosc/star_position_types.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*star_position_types.xosc', log)  is not None)

        # Check some scenario events

        self.assertTrue(re.search('^0.000: car_0_-1 New position:$\\n^0.000: Pos\\(55.00, -1.75, 0.00\\) Rot\\(3.14, 0.00, 0.00\\) roadId 0 laneId -1 offset 0.00 t -1.75', log, re.MULTILINE)  is not None)
        self.assertTrue(re.search('^0.400: event_road_position_rel_heading_0_-1_4 standbyState -> startTransition -> runningState', log, re.MULTILINE)  is not None)

        # Check vehicle key positions
        csv = generate_csv()

        self.assertTrue(re.search('^0.000, 0, car_0_-1, 55.000, -1.750, 0.000, 3.142, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 1, car_0_1, 55.000, 1.750, 0.000, 3.142, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 2, car_1_-1, 55.000, -1.750, -27.590, 3.142, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 3, car_1_1, 55.000, 1.750, -27.590, 3.142, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 4, car_2_-1, 55.000, -1.750, 27.590, 3.142, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 5, car_2_1, 55.000, 1.750, 27.590, 3.142, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 6, car_3_-1, 40.128, 37.653, 0.000, 3.927, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 7, car_3_1, 37.653, 40.128, 0.000, 3.927, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 8, car_4_-1, 40.128, 37.653, -27.590, 3.927, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 9, car_4_1, 37.653, 40.128, -27.590, 3.927, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 10, car_5_-1, 40.128, 37.653, 27.590, 3.927, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 11, car_5_1, 37.653, 40.128, 27.590, 3.927, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 12, car_6_-1, 1.750, 55.000, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 13, car_6_1, -1.750, 55.000, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 14, car_7_-1, 1.750, 55.000, -27.590, 4.712, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 15, car_7_1, -1.750, 55.000, -27.590, 4.712, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 16, car_8_-1, 1.750, 55.000, 27.590, 4.712, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 17, car_8_1, -1.750, 55.000, 27.590, 4.712, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 18, car_9_-1, -37.653, 40.128, 0.000, 5.498, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 19, car_9_1, -40.128, 37.653, 0.000, 5.498, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 20, car_10_-1, -37.653, 40.128, -27.590, 5.498, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 21, car_10_1, -40.128, 37.653, -27.590, 5.498, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 22, car_11_-1, -37.653, 40.128, 27.590, 5.498, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 23, car_11_1, -40.128, 37.653, 27.590, 5.498, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 24, car_12_-1, -55.000, 1.750, 0.000, 6.283, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 25, car_12_1, -55.000, -1.750, 0.000, 6.283, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 26, car_13_-1, -55.000, 1.750, -27.590, 6.283, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 27, car_13_1, -55.000, -1.750, -27.590, 6.283, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 28, car_14_-1, -55.000, 1.750, 27.590, 6.283, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 29, car_14_1, -55.000, -1.750, 27.590, 6.283, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 30, car_15_-1, -40.128, -37.653, 0.000, 0.785, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 31, car_15_1, -37.653, -40.128, 0.000, 0.785, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 32, car_16_-1, -40.128, -37.653, -27.590, 0.785, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 33, car_16_1, -37.653, -40.128, -27.590, 0.785, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 34, car_17_-1, -40.128, -37.653, 27.590, 0.785, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 35, car_17_1, -37.653, -40.128, 27.590, 0.785, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 36, car_18_-1, -1.750, -55.000, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 37, car_18_1, 1.750, -55.000, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 38, car_19_-1, -1.750, -55.000, -27.590, 1.571, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 39, car_19_1, 1.750, -55.000, -27.590, 1.571, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 40, car_20_-1, -1.750, -55.000, 27.590, 1.571, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 41, car_20_1, 1.750, -55.000, 27.590, 1.571, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 42, car_21_-1, 37.653, -40.128, 0.000, 2.356, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 43, car_21_1, 40.128, -37.653, 0.000, 2.356, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 44, car_22_-1, 37.653, -40.128, -27.590, 2.356, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 45, car_22_1, 40.128, -37.653, -27.590, 2.356, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 46, car_23_-1, 37.653, -40.128, 27.590, 2.356, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 47, car_23_1, 40.128, -37.653, 27.590, 2.356, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))

        self.assertTrue(re.search('^0.100, 0, car_0_-1, 55.000, -1.750, 0.000, 3.142, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 1, car_0_1, 55.000, 1.750, 0.000, 3.142, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 2, car_1_-1, 55.000, -1.750, -27.590, 3.142, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 3, car_1_1, 55.000, 1.750, -27.590, 3.142, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 4, car_2_-1, 55.000, -1.750, 27.590, 3.142, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 5, car_2_1, 55.000, 1.750, 27.590, 3.142, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 6, car_3_-1, 40.128, 37.653, 0.000, 3.927, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 7, car_3_1, 37.653, 40.128, 0.000, 3.927, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 8, car_4_-1, 40.128, 37.653, -27.590, 3.927, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 9, car_4_1, 37.653, 40.128, -27.590, 3.927, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 10, car_5_-1, 40.128, 37.653, 27.590, 3.927, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 11, car_5_1, 37.653, 40.128, 27.590, 3.927, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 12, car_6_-1, 1.750, 55.000, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 13, car_6_1, -1.750, 55.000, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 14, car_7_-1, 1.750, 55.000, -27.590, 4.712, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 15, car_7_1, -1.750, 55.000, -27.590, 4.712, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 16, car_8_-1, 1.750, 55.000, 27.590, 4.712, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 17, car_8_1, -1.750, 55.000, 27.590, 4.712, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 18, car_9_-1, -37.653, 40.128, 0.000, 5.498, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 19, car_9_1, -40.128, 37.653, 0.000, 5.498, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 20, car_10_-1, -37.653, 40.128, -27.590, 5.498, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 21, car_10_1, -40.128, 37.653, -27.590, 5.498, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 22, car_11_-1, -37.653, 40.128, 27.590, 5.498, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 23, car_11_1, -40.128, 37.653, 27.590, 5.498, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 24, car_12_-1, -55.000, 1.750, 0.000, 6.283, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 25, car_12_1, -55.000, -1.750, 0.000, 6.283, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 26, car_13_-1, -55.000, 1.750, -27.590, 6.283, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 27, car_13_1, -55.000, -1.750, -27.590, 6.283, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 28, car_14_-1, -55.000, 1.750, 27.590, 6.283, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 29, car_14_1, -55.000, -1.750, 27.590, 6.283, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 30, car_15_-1, -40.128, -37.653, 0.000, 0.785, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 31, car_15_1, -37.653, -40.128, 0.000, 0.785, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 32, car_16_-1, -40.128, -37.653, -27.590, 0.785, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 33, car_16_1, -37.653, -40.128, -27.590, 0.785, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 34, car_17_-1, -40.128, -37.653, 27.590, 0.785, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 35, car_17_1, -37.653, -40.128, 27.590, 0.785, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 36, car_18_-1, -1.750, -55.000, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 37, car_18_1, 1.750, -55.000, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 38, car_19_-1, -1.750, -55.000, -27.590, 1.571, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 39, car_19_1, 1.750, -55.000, -27.590, 1.571, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 40, car_20_-1, -1.750, -55.000, 27.590, 1.571, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 41, car_20_1, 1.750, -55.000, 27.590, 1.571, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 42, car_21_-1, 37.653, -40.128, 0.000, 2.356, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 43, car_21_1, 40.128, -37.653, 0.000, 2.356, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 44, car_22_-1, 37.653, -40.128, -27.590, 2.356, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 45, car_22_1, 40.128, -37.653, -27.590, 2.356, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 46, car_23_-1, 37.653, -40.128, 27.590, 2.356, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.100, 47, car_23_1, 40.128, -37.653, 27.590, 2.356, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))

        self.assertTrue(re.search('^0.200, 0, car_0_-1, 53.000, -1.750, 0.000, 3.142, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 1, car_0_1, 53.000, 1.750, 0.000, 3.142, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 2, car_1_-1, 53.000, -1.750, -26.364, 3.142, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 3, car_1_1, 53.000, 1.750, -26.364, 3.142, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 4, car_2_-1, 53.000, -1.750, 26.364, 3.142, 0.550, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 5, car_2_1, 53.000, 1.750, 26.364, 3.142, 0.550, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 6, car_3_-1, 38.714, 36.239, 0.000, 3.927, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 7, car_3_1, 36.239, 38.714, 0.000, 3.927, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 8, car_4_-1, 38.714, 36.239, -26.364, 3.927, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 9, car_4_1, 36.239, 38.714, -26.364, 3.927, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 10, car_5_-1, 38.714, 36.239, 26.364, 3.927, 0.550, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 11, car_5_1, 36.239, 38.714, 26.364, 3.927, 0.550, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 12, car_6_-1, 1.750, 53.000, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 13, car_6_1, -1.750, 53.000, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 14, car_7_-1, 1.750, 53.000, -26.364, 4.712, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 15, car_7_1, -1.750, 53.000, -26.364, 4.712, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 16, car_8_-1, 1.750, 53.000, 26.364, 4.712, 0.550, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 17, car_8_1, -1.750, 53.000, 26.364, 4.712, 0.550, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 18, car_9_-1, -36.239, 38.714, 0.000, 5.498, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 19, car_9_1, -38.714, 36.239, 0.000, 5.498, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 20, car_10_-1, -36.239, 38.714, -26.364, 5.498, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 21, car_10_1, -38.714, 36.239, -26.364, 5.498, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 22, car_11_-1, -36.239, 38.714, 26.364, 5.498, 0.550, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 23, car_11_1, -38.714, 36.239, 26.364, 5.498, 0.550, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 24, car_12_-1, -53.000, 1.750, 0.000, 6.283, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 25, car_12_1, -53.000, -1.750, 0.000, 6.283, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 26, car_13_-1, -53.000, 1.750, -26.364, 6.283, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 27, car_13_1, -53.000, -1.750, -26.364, 6.283, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 28, car_14_-1, -53.000, 1.750, 26.364, 6.283, 0.550, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 29, car_14_1, -53.000, -1.750, 26.364, 6.283, 0.550, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 30, car_15_-1, -38.714, -36.239, 0.000, 0.785, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 31, car_15_1, -36.239, -38.714, 0.000, 0.785, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 32, car_16_-1, -38.714, -36.239, -26.364, 0.785, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 33, car_16_1, -36.239, -38.714, -26.364, 0.785, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 34, car_17_-1, -38.714, -36.239, 26.364, 0.785, 0.550, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 35, car_17_1, -36.239, -38.714, 26.364, 0.785, 0.550, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 36, car_18_-1, -1.750, -53.000, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 37, car_18_1, 1.750, -53.000, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 38, car_19_-1, -1.750, -53.000, -26.364, 1.571, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 39, car_19_1, 1.750, -53.000, -26.364, 1.571, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 40, car_20_-1, -1.750, -53.000, 26.364, 1.571, 0.550, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 41, car_20_1, 1.750, -53.000, 26.364, 1.571, 0.550, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 42, car_21_-1, 36.239, -38.714, 0.000, 2.356, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 43, car_21_1, 38.714, -36.239, 0.000, 2.356, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 44, car_22_-1, 36.239, -38.714, -26.364, 2.356, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 45, car_22_1, 38.714, -36.239, -26.364, 2.356, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 46, car_23_-1, 36.239, -38.714, 26.364, 2.356, 0.550, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.200, 47, car_23_1, 38.714, -36.239, 26.364, 2.356, 0.550, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))

        self.assertTrue(re.search('^0.300, 0, car_0_-1, 51.000, -1.750, 0.000, 3.142, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 1, car_0_1, 51.000, 1.750, 0.000, 3.142, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 2, car_1_-1, 51.000, -1.750, -25.137, 3.142, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 3, car_1_1, 51.000, 1.750, -25.137, 3.142, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 4, car_2_-1, 51.000, -1.750, 25.137, 3.142, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 5, car_2_1, 51.000, 1.750, 25.137, 3.142, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 6, car_3_-1, 37.300, 34.825, 0.000, 3.927, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 7, car_3_1, 34.825, 37.300, 0.000, 3.927, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 8, car_4_-1, 37.300, 34.825, -25.137, 3.927, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 9, car_4_1, 34.825, 37.300, -25.137, 3.927, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 10, car_5_-1, 37.300, 34.825, 25.137, 3.927, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 11, car_5_1, 34.825, 37.300, 25.137, 3.927, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 12, car_6_-1, 1.750, 51.000, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 13, car_6_1, -1.750, 51.000, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 14, car_7_-1, 1.750, 51.000, -25.137, 4.712, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 15, car_7_1, -1.750, 51.000, -25.137, 4.712, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 16, car_8_-1, 1.750, 51.000, 25.137, 4.712, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 17, car_8_1, -1.750, 51.000, 25.137, 4.712, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 18, car_9_-1, -34.825, 37.300, 0.000, 5.498, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 19, car_9_1, -37.300, 34.825, 0.000, 5.498, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 20, car_10_-1, -34.825, 37.300, -25.137, 5.498, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 21, car_10_1, -37.300, 34.825, -25.137, 5.498, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 22, car_11_-1, -34.825, 37.300, 25.137, 5.498, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 23, car_11_1, -37.300, 34.825, 25.137, 5.498, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 24, car_12_-1, -51.000, 1.750, 0.000, 6.283, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 25, car_12_1, -51.000, -1.750, 0.000, 6.283, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 26, car_13_-1, -51.000, 1.750, -25.137, 6.283, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 27, car_13_1, -51.000, -1.750, -25.137, 6.283, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 28, car_14_-1, -51.000, 1.750, 25.137, 6.283, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 29, car_14_1, -51.000, -1.750, 25.137, 6.283, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 30, car_15_-1, -37.300, -34.825, 0.000, 0.785, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 31, car_15_1, -34.825, -37.300, 0.000, 0.785, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 32, car_16_-1, -37.300, -34.825, -25.137, 0.785, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 33, car_16_1, -34.825, -37.300, -25.137, 0.785, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 34, car_17_-1, -37.300, -34.825, 25.137, 0.785, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 35, car_17_1, -34.825, -37.300, 25.137, 0.785, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 36, car_18_-1, -1.750, -51.000, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 37, car_18_1, 1.750, -51.000, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 38, car_19_-1, -1.750, -51.000, -25.137, 1.571, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 39, car_19_1, 1.750, -51.000, -25.137, 1.571, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 40, car_20_-1, -1.750, -51.000, 25.137, 1.571, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 41, car_20_1, 1.750, -51.000, 25.137, 1.571, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 42, car_21_-1, 34.825, -37.300, 0.000, 2.356, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 43, car_21_1, 37.300, -34.825, 0.000, 2.356, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 44, car_22_-1, 34.825, -37.300, -25.137, 2.356, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 45, car_22_1, 37.300, -34.825, -25.137, 2.356, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 46, car_23_-1, 34.825, -37.300, 25.137, 2.356, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.300, 47, car_23_1, 37.300, -34.825, 25.137, 2.356, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))

        self.assertTrue(re.search('^0.400, 0, car_0_-1, 49.000, -1.750, 0.000, 3.142, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 1, car_0_1, 49.000, 1.750, 0.000, 3.142, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 2, car_1_-1, 49.000, -1.750, -23.911, 3.142, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 3, car_1_1, 49.000, 1.750, -23.911, 3.142, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 4, car_2_-1, 49.000, -1.750, 23.911, 3.142, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 5, car_2_1, 49.000, 1.750, 23.911, 3.142, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 6, car_3_-1, 35.886, 33.411, 0.000, 3.927, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 7, car_3_1, 33.411, 35.886, 0.000, 3.927, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 8, car_4_-1, 35.886, 33.411, -23.911, 3.927, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 9, car_4_1, 33.411, 35.886, -23.911, 3.927, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 10, car_5_-1, 35.886, 33.411, 23.911, 3.927, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 11, car_5_1, 33.411, 35.886, 23.911, 3.927, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 12, car_6_-1, 1.750, 49.000, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 13, car_6_1, -1.750, 49.000, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 14, car_7_-1, 1.750, 49.000, -23.911, 4.712, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 15, car_7_1, -1.750, 49.000, -23.911, 4.712, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 16, car_8_-1, 1.750, 49.000, 23.911, 4.712, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 17, car_8_1, -1.750, 49.000, 23.911, 4.712, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 18, car_9_-1, -33.411, 35.886, 0.000, 5.498, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 19, car_9_1, -35.886, 33.411, 0.000, 5.498, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 20, car_10_-1, -33.411, 35.886, -23.911, 5.498, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 21, car_10_1, -35.886, 33.411, -23.911, 5.498, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 22, car_11_-1, -33.411, 35.886, 23.911, 5.498, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 23, car_11_1, -35.886, 33.411, 23.911, 5.498, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 24, car_12_-1, -49.000, 1.750, 0.000, 6.283, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 25, car_12_1, -49.000, -1.750, 0.000, 6.283, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 26, car_13_-1, -49.000, 1.750, -23.911, 6.283, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 27, car_13_1, -49.000, -1.750, -23.911, 6.283, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 28, car_14_-1, -49.000, 1.750, 23.911, 6.283, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 29, car_14_1, -49.000, -1.750, 23.911, 6.283, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 30, car_15_-1, -35.886, -33.411, 0.000, 0.785, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 31, car_15_1, -33.411, -35.886, 0.000, 0.785, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 32, car_16_-1, -35.886, -33.411, -23.911, 0.785, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 33, car_16_1, -33.411, -35.886, -23.911, 0.785, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 34, car_17_-1, -35.886, -33.411, 23.911, 0.785, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 35, car_17_1, -33.411, -35.886, 23.911, 0.785, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 36, car_18_-1, -1.750, -49.000, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 37, car_18_1, 1.750, -49.000, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 38, car_19_-1, -1.750, -49.000, -23.911, 1.571, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 39, car_19_1, 1.750, -49.000, -23.911, 1.571, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 40, car_20_-1, -1.750, -49.000, 23.911, 1.571, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 41, car_20_1, 1.750, -49.000, 23.911, 1.571, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 42, car_21_-1, 33.411, -35.886, 0.000, 2.356, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 43, car_21_1, 35.886, -33.411, 0.000, 2.356, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 44, car_22_-1, 33.411, -35.886, -23.911, 2.356, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 45, car_22_1, 35.886, -33.411, -23.911, 2.356, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 46, car_23_-1, 33.411, -35.886, 23.911, 2.356, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.400, 47, car_23_1, 35.886, -33.411, 23.911, 2.356, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))

        self.assertTrue(re.search('^0.500, 0, car_0_-1, 47.000, -1.750, 0.000, 3.142, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 1, car_0_1, 47.000, 1.750, 0.000, 3.142, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 2, car_1_-1, 47.000, -1.750, -22.685, 3.142, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 3, car_1_1, 47.000, 1.750, -22.685, 3.142, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 4, car_2_-1, 47.000, -1.750, 22.685, 3.142, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 5, car_2_1, 47.000, 1.750, 22.685, 3.142, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 6, car_3_-1, 34.471, 31.997, 0.000, 3.927, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 7, car_3_1, 31.997, 34.471, 0.000, 3.927, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 8, car_4_-1, 34.471, 31.997, -22.685, 3.927, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 9, car_4_1, 31.997, 34.471, -22.685, 3.927, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 10, car_5_-1, 34.471, 31.997, 22.685, 3.927, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 11, car_5_1, 31.997, 34.471, 22.685, 3.927, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 12, car_6_-1, 1.750, 47.000, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 13, car_6_1, -1.750, 47.000, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 14, car_7_-1, 1.750, 47.000, -22.685, 4.712, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 15, car_7_1, -1.750, 47.000, -22.685, 4.712, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 16, car_8_-1, 1.750, 47.000, 22.685, 4.712, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 17, car_8_1, -1.750, 47.000, 22.685, 4.712, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 18, car_9_-1, -31.997, 34.471, 0.000, 5.498, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 19, car_9_1, -34.471, 31.997, 0.000, 5.498, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 20, car_10_-1, -31.997, 34.471, -22.685, 5.498, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 21, car_10_1, -34.471, 31.997, -22.685, 5.498, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 22, car_11_-1, -31.997, 34.471, 22.685, 5.498, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 23, car_11_1, -34.471, 31.997, 22.685, 5.498, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 24, car_12_-1, -47.000, 1.750, 0.000, 6.283, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 25, car_12_1, -47.000, -1.750, 0.000, 6.283, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 26, car_13_-1, -47.000, 1.750, -22.685, 6.283, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 27, car_13_1, -47.000, -1.750, -22.685, 6.283, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 28, car_14_-1, -47.000, 1.750, 22.685, 6.283, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 29, car_14_1, -47.000, -1.750, 22.685, 6.283, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 30, car_15_-1, -34.471, -31.997, 0.000, 0.785, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 31, car_15_1, -31.997, -34.471, 0.000, 0.785, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 32, car_16_-1, -34.471, -31.997, -22.685, 0.785, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 33, car_16_1, -31.997, -34.471, -22.685, 0.785, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 34, car_17_-1, -34.471, -31.997, 22.685, 0.785, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 35, car_17_1, -31.997, -34.471, 22.685, 0.785, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 36, car_18_-1, -1.750, -47.000, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 37, car_18_1, 1.750, -47.000, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 38, car_19_-1, -1.750, -47.000, -22.685, 1.571, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 39, car_19_1, 1.750, -47.000, -22.685, 1.571, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 40, car_20_-1, -1.750, -47.000, 22.685, 1.571, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 41, car_20_1, 1.750, -47.000, 22.685, 1.571, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 42, car_21_-1, 31.997, -34.471, 0.000, 2.356, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 43, car_21_1, 34.471, -31.997, 0.000, 2.356, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 44, car_22_-1, 31.997, -34.471, -22.685, 2.356, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 45, car_22_1, 34.471, -31.997, -22.685, 2.356, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 46, car_23_-1, 31.997, -34.471, 22.685, 2.356, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.500, 47, car_23_1, 34.471, -31.997, 22.685, 2.356, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))

        self.assertTrue(re.search('^0.600, 0, car_0_-1, 45.000, -1.750, 0.000, 3.142, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 1, car_0_1, 45.000, 1.750, 0.000, 3.142, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 2, car_1_-1, 45.000, -1.750, -21.459, 3.142, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 3, car_1_1, 45.000, 1.750, -21.459, 3.142, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 4, car_2_-1, 45.000, -1.750, 21.459, 3.142, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 5, car_2_1, 45.000, 1.750, 21.459, 3.142, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 6, car_3_-1, 33.057, 30.582, 0.000, 3.927, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 7, car_3_1, 30.582, 33.057, 0.000, 3.927, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 8, car_4_-1, 33.057, 30.582, -21.459, 3.927, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 9, car_4_1, 30.582, 33.057, -21.459, 3.927, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 10, car_5_-1, 33.057, 30.582, 21.459, 3.927, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 11, car_5_1, 30.582, 33.057, 21.459, 3.927, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 12, car_6_-1, 1.750, 45.000, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 13, car_6_1, -1.750, 45.000, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 14, car_7_-1, 1.750, 45.000, -21.459, 4.712, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 15, car_7_1, -1.750, 45.000, -21.459, 4.712, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 16, car_8_-1, 1.750, 45.000, 21.459, 4.712, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 17, car_8_1, -1.750, 45.000, 21.459, 4.712, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 18, car_9_-1, -30.582, 33.057, 0.000, 5.498, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 19, car_9_1, -33.057, 30.582, 0.000, 5.498, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 20, car_10_-1, -30.582, 33.057, -21.459, 5.498, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 21, car_10_1, -33.057, 30.582, -21.459, 5.498, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 22, car_11_-1, -30.582, 33.057, 21.459, 5.498, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 23, car_11_1, -33.057, 30.582, 21.459, 5.498, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 24, car_12_-1, -45.000, 1.750, 0.000, 6.283, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 25, car_12_1, -45.000, -1.750, 0.000, 6.283, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 26, car_13_-1, -45.000, 1.750, -21.459, 6.283, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 27, car_13_1, -45.000, -1.750, -21.459, 6.283, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 28, car_14_-1, -45.000, 1.750, 21.459, 6.283, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 29, car_14_1, -45.000, -1.750, 21.459, 6.283, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 30, car_15_-1, -33.057, -30.582, 0.000, 0.785, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 31, car_15_1, -30.582, -33.057, 0.000, 0.785, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 32, car_16_-1, -33.057, -30.582, -21.459, 0.785, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 33, car_16_1, -30.582, -33.057, -21.459, 0.785, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 34, car_17_-1, -33.057, -30.582, 21.459, 0.785, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 35, car_17_1, -30.582, -33.057, 21.459, 0.785, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 36, car_18_-1, -1.750, -45.000, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 37, car_18_1, 1.750, -45.000, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 38, car_19_-1, -1.750, -45.000, -21.459, 1.571, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 39, car_19_1, 1.750, -45.000, -21.459, 1.571, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 40, car_20_-1, -1.750, -45.000, 21.459, 1.571, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 41, car_20_1, 1.750, -45.000, 21.459, 1.571, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 42, car_21_-1, 30.582, -33.057, 0.000, 2.356, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 43, car_21_1, 33.057, -30.582, 0.000, 2.356, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 44, car_22_-1, 30.582, -33.057, -21.459, 2.356, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 45, car_22_1, 33.057, -30.582, -21.459, 2.356, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 46, car_23_-1, 30.582, -33.057, 21.459, 2.356, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.600, 47, car_23_1, 33.057, -30.582, 21.459, 2.356, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))

        self.assertTrue(re.search('^0.700, 0, car_0_-1, 44.000, -1.750, 0.000, 3.142, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 1, car_0_1, 42.000, 1.750, 0.000, 3.142, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 2, car_1_-1, 44.000, -1.750, -20.846, 3.142, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 3, car_1_1, 42.000, 1.750, -19.619, 3.142, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 4, car_2_-1, 44.000, -1.750, 20.846, 3.142, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 5, car_2_1, 42.000, 1.750, 19.619, 3.142, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 6, car_3_-1, 32.350, 29.875, 0.000, 3.927, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 7, car_3_1, 28.461, 30.936, 0.000, 3.927, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 8, car_4_-1, 32.350, 29.875, -20.846, 3.927, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 9, car_4_1, 28.461, 30.936, -19.619, 3.927, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 10, car_5_-1, 32.350, 29.875, 20.846, 3.927, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 11, car_5_1, 28.461, 30.936, 19.619, 3.927, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 12, car_6_-1, 1.750, 44.000, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 13, car_6_1, -1.750, 42.000, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 14, car_7_-1, 1.750, 44.000, -20.846, 4.712, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 15, car_7_1, -1.750, 42.000, -19.619, 4.712, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 16, car_8_-1, 1.750, 44.000, 20.846, 4.712, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 17, car_8_1, -1.750, 42.000, 19.619, 4.712, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 18, car_9_-1, -29.875, 32.350, 0.000, 5.498, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 19, car_9_1, -30.936, 28.461, 0.000, 5.498, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 20, car_10_-1, -29.875, 32.350, -20.846, 5.498, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 21, car_10_1, -30.936, 28.461, -19.619, 5.498, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 22, car_11_-1, -29.875, 32.350, 20.846, 5.498, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 23, car_11_1, -30.936, 28.461, 19.619, 5.498, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 24, car_12_-1, -44.000, 1.750, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 25, car_12_1, -42.000, -1.750, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 26, car_13_-1, -44.000, 1.750, -20.846, 0.000, 5.733, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 27, car_13_1, -42.000, -1.750, -19.619, 0.000, 5.733, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 28, car_14_-1, -44.000, 1.750, 20.846, 0.000, 0.550, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 29, car_14_1, -42.000, -1.750, 19.619, 0.000, 0.550, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 30, car_15_-1, -32.350, -29.875, 0.000, 0.785, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 31, car_15_1, -28.461, -30.936, 0.000, 0.785, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 32, car_16_-1, -32.350, -29.875, -20.846, 0.785, 5.733, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 33, car_16_1, -28.461, -30.936, -19.619, 0.785, 5.733, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 34, car_17_-1, -32.350, -29.875, 20.846, 0.785, 0.550, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 35, car_17_1, -28.461, -30.936, 19.619, 0.785, 0.550, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 36, car_18_-1, -1.750, -44.000, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 37, car_18_1, 1.750, -42.000, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 38, car_19_-1, -1.750, -44.000, -20.846, 1.571, 5.733, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 39, car_19_1, 1.750, -42.000, -19.619, 1.571, 5.733, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 40, car_20_-1, -1.750, -44.000, 20.846, 1.571, 0.550, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 41, car_20_1, 1.750, -42.000, 19.619, 1.571, 0.550, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 42, car_21_-1, 29.875, -32.350, 0.000, 2.356, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 43, car_21_1, 30.936, -28.461, 0.000, 2.356, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 44, car_22_-1, 29.875, -32.350, -20.846, 2.356, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 45, car_22_1, 30.936, -28.461, -19.619, 2.356, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 46, car_23_-1, 29.875, -32.350, 20.846, 2.356, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.700, 47, car_23_1, 30.936, -28.461, 19.619, 2.356, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))

        self.assertTrue(re.search('^0.800, 0, car_0_-1, 41.000, 1.750, 0.000, 3.142, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 1, car_0_1, 39.000, -1.750, 0.000, 3.142, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 2, car_1_-1, 41.000, 1.750, -19.006, 3.142, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 3, car_1_1, 39.000, -1.750, -17.780, 3.142, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 4, car_2_-1, 41.000, 1.750, 19.006, 3.142, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 5, car_2_1, 39.000, -1.750, 17.780, 3.142, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 6, car_3_-1, 27.754, 30.229, 0.000, 3.927, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 7, car_3_1, 28.815, 26.340, 0.000, 3.927, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 8, car_4_-1, 27.754, 30.229, -19.006, 3.927, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 9, car_4_1, 28.815, 26.340, -17.780, 3.927, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 10, car_5_-1, 27.754, 30.229, 19.006, 3.927, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 11, car_5_1, 28.815, 26.340, 17.780, 3.927, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 12, car_6_-1, -1.750, 41.000, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 13, car_6_1, 1.750, 39.000, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 14, car_7_-1, -1.750, 41.000, -19.006, 4.712, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 15, car_7_1, 1.750, 39.000, -17.780, 4.712, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 16, car_8_-1, -1.750, 41.000, 19.006, 4.712, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 17, car_8_1, 1.750, 39.000, 17.780, 4.712, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 18, car_9_-1, -30.229, 27.754, 0.000, 5.498, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 19, car_9_1, -26.340, 28.815, 0.000, 5.498, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 20, car_10_-1, -30.229, 27.754, -19.006, 5.498, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 21, car_10_1, -26.340, 28.815, -17.780, 5.498, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 22, car_11_-1, -30.229, 27.754, 19.006, 5.498, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 23, car_11_1, -26.340, 28.815, 17.780, 5.498, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 24, car_12_-1, -41.000, -1.750, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 25, car_12_1, -39.000, 1.750, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 26, car_13_-1, -41.000, -1.750, -19.006, 0.000, 5.733, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 27, car_13_1, -39.000, 1.750, -17.780, 0.000, 5.733, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 28, car_14_-1, -41.000, -1.750, 19.006, 0.000, 0.550, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 29, car_14_1, -39.000, 1.750, 17.780, 0.000, 0.550, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 30, car_15_-1, -27.754, -30.229, 0.000, 0.785, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 31, car_15_1, -28.815, -26.340, 0.000, 0.785, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 32, car_16_-1, -27.754, -30.229, -19.006, 0.785, 5.733, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 33, car_16_1, -28.815, -26.340, -17.780, 0.785, 5.733, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 34, car_17_-1, -27.754, -30.229, 19.006, 0.785, 0.550, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 35, car_17_1, -28.815, -26.340, 17.780, 0.785, 0.550, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 36, car_18_-1, 1.750, -41.000, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 37, car_18_1, -1.750, -39.000, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 38, car_19_-1, 1.750, -41.000, -19.006, 1.571, 5.733, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 39, car_19_1, -1.750, -39.000, -17.780, 1.571, 5.733, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 40, car_20_-1, 1.750, -41.000, 19.006, 1.571, 0.550, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 41, car_20_1, -1.750, -39.000, 17.780, 1.571, 0.550, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 42, car_21_-1, 30.229, -27.754, 0.000, 2.356, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 43, car_21_1, 26.340, -28.815, 0.000, 2.356, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 44, car_22_-1, 30.229, -27.754, -19.006, 2.356, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 45, car_22_1, 26.340, -28.815, -17.780, 2.356, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 46, car_23_-1, 30.229, -27.754, 19.006, 2.356, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.800, 47, car_23_1, 26.340, -28.815, 17.780, 2.356, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))

        self.assertTrue(re.search('^0.900, 0, car_0_-1, 39.000, 1.750, 0.000, 3.142, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 1, car_0_1, 37.000, -1.750, 0.000, 3.142, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 2, car_1_-1, 39.000, 1.750, -17.780, 3.142, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 3, car_1_1, 37.000, -1.750, -16.554, 3.142, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 4, car_2_-1, 39.000, 1.750, 17.780, 3.142, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 5, car_2_1, 37.000, -1.750, 16.554, 3.142, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 6, car_3_-1, 26.340, 28.815, 0.000, 3.927, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 7, car_3_1, 27.400, 24.926, 0.000, 3.927, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 8, car_4_-1, 26.340, 28.815, -17.780, 3.927, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 9, car_4_1, 27.400, 24.926, -16.554, 3.927, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 10, car_5_-1, 26.340, 28.815, 17.780, 3.927, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 11, car_5_1, 27.400, 24.926, 16.554, 3.927, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 12, car_6_-1, -1.750, 39.000, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 13, car_6_1, 1.750, 37.000, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 14, car_7_-1, -1.750, 39.000, -17.780, 4.712, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 15, car_7_1, 1.750, 37.000, -16.554, 4.712, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 16, car_8_-1, -1.750, 39.000, 17.780, 4.712, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 17, car_8_1, 1.750, 37.000, 16.554, 4.712, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 18, car_9_-1, -28.815, 26.340, 0.000, 5.498, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 19, car_9_1, -24.926, 27.400, 0.000, 5.498, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 20, car_10_-1, -28.815, 26.340, -17.780, 5.498, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 21, car_10_1, -24.926, 27.400, -16.554, 5.498, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 22, car_11_-1, -28.815, 26.340, 17.780, 5.498, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 23, car_11_1, -24.926, 27.400, 16.554, 5.498, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 24, car_12_-1, -39.000, -1.750, 0.000, 6.283, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 25, car_12_1, -37.000, 1.750, 0.000, 6.283, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 26, car_13_-1, -39.000, -1.750, -17.780, 6.283, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 27, car_13_1, -37.000, 1.750, -16.554, 6.283, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 28, car_14_-1, -39.000, -1.750, 17.780, 6.283, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 29, car_14_1, -37.000, 1.750, 16.554, 6.283, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 30, car_15_-1, -26.340, -28.815, 0.000, 0.785, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 31, car_15_1, -27.400, -24.926, 0.000, 0.785, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 32, car_16_-1, -26.340, -28.815, -17.780, 0.785, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 33, car_16_1, -27.400, -24.926, -16.554, 0.785, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 34, car_17_-1, -26.340, -28.815, 17.780, 0.785, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 35, car_17_1, -27.400, -24.926, 16.554, 0.785, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 36, car_18_-1, 1.750, -39.000, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 37, car_18_1, -1.750, -37.000, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 38, car_19_-1, 1.750, -39.000, -17.780, 1.571, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 39, car_19_1, -1.750, -37.000, -16.554, 1.571, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 40, car_20_-1, 1.750, -39.000, 17.780, 1.571, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 41, car_20_1, -1.750, -37.000, 16.554, 1.571, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 42, car_21_-1, 28.815, -26.340, 0.000, 2.356, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 43, car_21_1, 24.926, -27.400, 0.000, 2.356, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 44, car_22_-1, 28.815, -26.340, -17.780, 2.356, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 45, car_22_1, 24.926, -27.400, -16.554, 2.356, 5.733, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 46, car_23_-1, 28.815, -26.340, 17.780, 2.356, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.900, 47, car_23_1, 24.926, -27.400, 16.554, 2.356, 0.550, 6.283, 0.000, 0.000, 0.000', csv, re.MULTILINE))

    def test_alks(self):
        # Idea: Run scenario several times with different ALKS safety models

        with open('../resources/xosc/alks_r157_cut_in_quick_brake.xosc') as f:
            xosc_str = f.read()

        args = '--headless --fixed_timestep 0.01 --path ../resources/xodr --path ../resources/xosc/Catalogs/Vehicles'
        models = ["ReferenceDriver", "Regulation", "FSM", "RSS"]
        log = []

        for model in models:
            # Modify controller model name property
            tmp_xosc_str = re.sub('<Property name="model" value=.*/>', '<Property name="model" value=\"' + model + '\"/>', xosc_str)

            if model == "ReferenceDriver":
                tmp_xosc_str = re.sub('<Property name="cruise" value=.*/>', '<Property name="cruise" value=\"' + "false" + '\"/>', tmp_xosc_str)
            else:
                tmp_xosc_str = re.sub('<Property name="cruise" value=.*/>', '<Property name="cruise" value=\"' + "true" + '\"/>', tmp_xosc_str)

            # Save in separate .dat file
            log.append(run_scenario(None, args + ' --record sim_model_' + model + '.dat', tmp_xosc_str))

            # Verify that the scenario was executed as expected
            self.assertTrue(re.search('^Loading inline', log[-1], re.MULTILINE)  is not None)
            self.assertTrue(re.search('^0.000: Recording data to file sim_', log[-1], re.MULTILINE)  is not None)
            self.assertTrue(re.search('^0.000: Controller ALKS_R157SM_Controller activated \\(lat OFF, long ON\\), domain mask=0x1', log[-1], re.MULTILINE)  is not None)

        if len(models) > 0:
            with open(STDOUT_FILENAME, "w") as f:
                if len(models) > 1:
                    # merge dat files (identify by 'sim_model_*')
                    args = [os.path.join(ESMINI_PATH,'bin','replayer'), '--res_path', '../resources', '--dir', '.', '--file', 'sim_model_', '--save_merged', 'sim.dat']
                    process = subprocess.Popen(args, cwd=os.path.dirname(os.path.realpath(__file__)), stdout=f)
                    assert process.wait() == 0
                else:
                    os.replace('sim_model_' + models[0] + '.dat', 'sim.dat')

        # Check vehicle key positions
        csv = generate_csv()

        self.assertTrue(re.search('^0.000, 0, Ego, 50.000, -1.535, 0.000, 0.000, 0.000, 0.000, 20.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 1, Target, 80.000, 1.535, 0.000, 0.000, 0.000, 0.000, 15.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 100, Ego, 50.000, -1.535, 0.000, 0.000, 0.000, 0.000, 20.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 101, Target, 80.000, 1.535, 0.000, 0.000, 0.000, 0.000, 15.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 200, Ego, 50.000, -1.535, 0.000, 0.000, 0.000, 0.000, 20.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 201, Target, 80.000, 1.535, 0.000, 0.000, 0.000, 0.000, 15.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 300, Ego, 50.000, -1.535, 0.000, 0.000, 0.000, 0.000, 20.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 301, Target, 80.000, 1.535, 0.000, 0.000, 0.000, 0.000, 15.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^2.840, 0, Ego, 106.800, -1.535, 0.000, 0.000, 0.000, 0.000, 20.000, 0.000, 5.206', csv, re.MULTILINE))
        self.assertTrue(re.search('^2.840, 1, Target, 122.585, 0.991, 0.000, 6.202, 0.000, 0.000, 15.000, -0.015, 2.334', csv, re.MULTILINE))
        self.assertTrue(re.search('^2.840, 100, Ego, 106.800, -1.535, 0.000, 0.000, 0.000, 0.000, 19.999, 0.000, 5.206', csv, re.MULTILINE))
        self.assertTrue(re.search('^2.840, 101, Target, 122.585, 0.991, 0.000, 6.202, 0.000, 0.000, 15.000, -0.015, 2.334', csv, re.MULTILINE))
        self.assertTrue(re.search('^2.840, 200, Ego, 106.800, -1.535, 0.000, 0.000, 0.000, 0.000, 20.000, 0.000, 5.206', csv, re.MULTILINE))
        self.assertTrue(re.search('^2.840, 201, Target, 122.585, 0.991, 0.000, 6.202, 0.000, 0.000, 15.000, -0.015, 2.334', csv, re.MULTILINE))
        self.assertTrue(re.search('^2.840, 300, Ego, 106.800, -1.535, 0.000, 0.000, 0.000, 0.000, 20.000, 0.000, 5.206', csv, re.MULTILINE))
        self.assertTrue(re.search('^2.840, 301, Target, 122.585, 0.991, 0.000, 6.202, 0.000, 0.000, 15.000, -0.015, 2.334', csv, re.MULTILINE))
        self.assertTrue(re.search('^2.850, 0, Ego, 107.000, -1.535, 0.000, 0.000, 0.000, 0.000, 20.000, 0.000, 5.778', csv, re.MULTILINE))
        self.assertTrue(re.search('^2.850, 1, Target, 122.734, 0.978, 0.000, 6.201, 0.000, 0.000, 15.000, -0.015, 2.762', csv, re.MULTILINE))
        self.assertTrue(re.search('^2.850, 100, Ego, 107.000, -1.535, 0.000, 0.000, 0.000, 0.000, 19.996, 0.000, 5.777', csv, re.MULTILINE))
        self.assertTrue(re.search('^2.850, 101, Target, 122.734, 0.978, 0.000, 6.201, 0.000, 0.000, 15.000, -0.015, 2.762', csv, re.MULTILINE))
        self.assertTrue(re.search('^2.850, 200, Ego, 107.000, -1.535, 0.000, 0.000, 0.000, 0.000, 20.000, 0.000, 5.778', csv, re.MULTILINE))
        self.assertTrue(re.search('^2.850, 201, Target, 122.734, 0.978, 0.000, 6.201, 0.000, 0.000, 15.000, -0.015, 2.762', csv, re.MULTILINE))
        self.assertTrue(re.search('^2.850, 300, Ego, 107.000, -1.535, 0.000, 0.000, 0.000, 0.000, 20.000, 0.000, 5.778', csv, re.MULTILINE))
        self.assertTrue(re.search('^2.850, 301, Target, 122.734, 0.978, 0.000, 6.201, 0.000, 0.000, 15.000, -0.015, 2.762', csv, re.MULTILINE))
        self.assertTrue(re.search('^4.990, 300, Ego, 140.769, -1.535, 0.000, 0.000, 0.000, 0.000, 11.520, 0.000, 1.729', csv, re.MULTILINE))
        self.assertTrue(re.search('^4.990, 301, Target, 152.246, -1.535, 0.000, 6.280, 0.000, 0.000, 9.904, 0.053, 5.588', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.350, 0, Ego, 155.824, -1.535, 0.000, 0.000, 0.000, 0.000, 6.321, 0.000, 0.763', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.350, 1, Target, 160.872, -1.535, 0.000, 0.000, 0.000, 0.000, 2.832, 0.000, 5.098', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.350, 100, Ego, 148.061, -1.535, 0.000, 0.000, 0.000, 0.000, 4.393, 0.000, 3.713', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.350, 101, Target, 160.872, -1.535, 0.000, 0.000, 0.000, 0.000, 2.832, 0.000, 5.098', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.350, 300, Ego, 152.710, -1.535, 0.000, 0.000, 0.000, 0.000, 6.080, 0.000, 4.429', csv, re.MULTILINE))
        self.assertTrue(re.search('^6.350, 301, Target, 160.872, -1.535, 0.000, 0.000, 0.000, 0.000, 2.832, 0.000, 5.098', csv, re.MULTILINE))

    def test_user_defined_action(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'EnvironmentSimulator/Unittest/xosc/user_defined_action.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*user_defined_action.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('4.890: Starting UserDefinedAction type: noop content: Wait', log)  is not None)
        self.assertTrue(re.search('\\n7.000: Event WaitEvent ended, overwritten by event DecelerateEvent', log)  is not None)
        self.assertTrue(re.search('\\n9.950: DecelerateDoneCondition timer expired at 1.00 seconds', log)  is not None)

        # Check vehicle key positions
        csv = generate_csv()

        self.assertTrue(re.search('4.880, 0, Ego, 57.733, -1.535, 0.000, 0.000, 0.000, 0.000, 19.400, 0.000, 0.994\\n', csv))
        self.assertTrue(re.search('4.890, 0, Ego, 57.927, -1.535, 0.000, 0.000, 0.000, 0.000, 19.444, 0.000, 1.550\\n', csv))
        self.assertTrue(re.search('\\n7.000, 0, Ego, 98.955, -1.535, 0.000, 0.000, 0.000, 0.000, 19.444, 0.000, 5.675', csv))
        self.assertTrue(re.search('\\n7.010, 0, Ego, 99.149, -1.535, 0.000, 0.000, 0.000, 0.000, 19.344, 0.000, 6.228', csv))
        self.assertTrue(re.search('\\n8.940, 0, Ego, 117.762, -1.535, 0.000, 0.000, 0.000, 0.000, 0.044, 0.000, 2.861', csv))
        self.assertTrue(re.search('\\n8.950, 0, Ego, 117.762, -1.535, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 2.861', csv))

    def test_trailer_connect(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/trailer_connect.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*trailer_connect.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('2.000: ReverseTrigger == true, 2.0000 >= 1.00 edge: none', log)  is not None)
        self.assertTrue(re.search('8.720: ConnectEvent complete after 1 execution', log)  is not None)
        self.assertTrue(re.search('18.980: DisconnectEvent == true, element: ForwardEvent state: END_TRANSITION, edge: rising', log)  is not None)

        # Check vehicle key positions
        csv = generate_csv()

        self.assertTrue(re.search('^0.000, 0, Trailer, 1.700, 13.500, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 1, Car, -10.300, 30.200, 0.000, 3.142, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^5.600, 0, Trailer, 1.700, 13.500, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^5.600, 1, Car, 0.884, 22.547, 0.000, 1.942, 0.000, 0.000, -4.000, 0.262, -3.444', csv, re.MULTILINE))
        self.assertTrue(re.search('^7.800, 0, Trailer, 1.700, 13.500, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^7.800, 1, Car, 1.699, 18.149, 0.000, 1.571, 0.000, 0.000, -4.000, 0.000, -3.454', csv, re.MULTILINE))
        self.assertTrue(re.search('^9.500, 0, Trailer, 1.700, 13.699, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, -2.631', csv, re.MULTILINE))
        self.assertTrue(re.search('^9.500, 1, Car, 1.699, 18.149, 0.000, 1.571, 0.000, 0.000, -4.000, 0.000, -4.033', csv, re.MULTILINE))
        self.assertTrue(re.search('^9.500, 1, Car, 1.699, 18.149, 0.000, 1.571, 0.000, 0.000, -4.000, 0.000, -4.033', csv, re.MULTILINE))
        self.assertTrue(re.search('^9.500, 1, Car, 1.699, 18.149, 0.000, 1.571, 0.000, 0.000, -4.000, 0.000, -4.033', csv, re.MULTILINE))
        self.assertTrue(re.search('^14.600, 0, Trailer, 6.460, 27.561, 0.000, 0.716, 0.000, 0.000, 0.000, -0.224, 2.408', csv, re.MULTILINE))
        self.assertTrue(re.search('^14.600, 1, Car, 10.017, 30.162, 0.000, 0.401, 0.000, 0.000, 4.000, -0.213, 1.006', csv, re.MULTILINE))
        self.assertTrue(re.search('^17.500, 0, Trailer, 6.849, 27.886, 0.000, 0.679, 0.000, 0.000, 4.000, 0.000, 4.135', csv, re.MULTILINE))
        self.assertTrue(re.search('^17.500, 1, Car, 19.354, 31.029, 0.000, 6.083, 0.000, 0.000, 4.000, 0.000, 2.733', csv, re.MULTILINE))

    def test_pedestrian(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/pedestrian.xosc'), COMMON_ARGS + "--fixed_timestep 0.1")

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*pedestrian.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('0.600: ped_walk_event == true, traveled_dist: 6.00 >= 5.00, edge: rising', log)  is not None)
        self.assertTrue(re.search('3.800: brake_Condition == true, TTC: 1.20 < 1.20, edge rising', log)  is not None)
        self.assertTrue(re.search('14.400: QuitCondition == true, distance 4.87 < tolerance \\(5.00\\), edge: rising', log)  is not None)

        # Check vehicle key positions
        csv = generate_csv()

        self.assertTrue(re.search('^0.000, 0, Ego, 42.984, -71.249, 0.000, 1.776, 0.000, 0.000, 10.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^0.000, 1, pedestrian_adult, 35.692, -23.629, 0.000, 1.798, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^3.900, 0, Ego, 34.307, -33.282, 0.000, 1.803, 0.000, 0.000, 9.490, -0.001, 4.469', csv, re.MULTILINE))
        self.assertTrue(re.search('^3.900, 1, pedestrian_adult, 34.693, -19.282, 0.000, 1.795, 0.000, 0.000, 1.500, -0.003, 0.176', csv, re.MULTILINE))
        self.assertTrue(re.search('^4.100, 1, pedestrian_adult, 34.452, -19.114, 0.000, 2.415, 0.000, 0.000, 1.500, 0.097, 1.034', csv, re.MULTILINE))
        self.assertTrue(re.search('^4.200, 1, pedestrian_adult, 34.325, -19.034, 0.000, 2.749, 0.000, 0.000, 1.500, 0.147, 1.462', csv, re.MULTILINE))
        self.assertTrue(re.search('^4.300, 1, pedestrian_adult, 34.198, -18.955, 0.000, 3.083, 0.000, 0.000, 1.500, 0.197, 1.891', csv, re.MULTILINE))
        self.assertTrue(re.search('^4.400, 1, pedestrian_adult, 34.067, -18.893, 0.000, 3.364, 0.000, 0.000, 1.500, 0.247, 2.319', csv, re.MULTILINE))
        self.assertTrue(re.search('^4.500, 1, pedestrian_adult, 33.921, -18.926, 0.000, 3.364, 0.000, 0.000, 1.500, 0.197, 2.748', csv, re.MULTILINE))
        self.assertTrue(re.search('^5.600, 0, Ego, 32.406, -25.172, 0.000, 1.799, 0.000, 0.000, 0.820, -0.002, 3.136', csv, re.MULTILINE))
        self.assertTrue(re.search('^5.700, 0, Ego, 32.399, -25.142, 0.000, 1.799, 0.000, 0.000, 0.310, -0.001, 3.225', csv, re.MULTILINE))
        self.assertTrue(re.search('^11.000, 1, pedestrian_adult, 24.456, -20.843, 0.000, 2.646, 0.000, 0.000, 1.500, -0.151, 5.472', csv, re.MULTILINE))
        self.assertTrue(re.search('^11.300, 1, pedestrian_adult, 24.116, -20.574, 0.000, 1.792, 0.000, 0.000, 1.500, -0.301, 0.475', csv, re.MULTILINE))

    def test_drive_when_close(self):
        # this test case exercises restarting events within same maneuver. A car will drive only when near another specific one, otherwise stop.

        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/drive_when_close.xosc'), COMMON_ARGS + "--fixed_timestep 0.1 --disable_controllers")

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*drive_when_close.xosc', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('6.000: DriveCondition == true, rel_dist: 29.50 < 30.00, edge: rising', log)  is not None)
        self.assertTrue(re.search('9.900: SpeedAction runningState -> endTransition -> completeState', log)  is not None)
        self.assertTrue(re.search('10.000: StopCondition == true, rel_dist: 30.50 > 30.00, edge: rising', log)  is not None)
        self.assertTrue(re.search('30.000: DriveCondition == true, rel_dist: 29.50 < 30.00, edge: rising', log)  is not None)
        self.assertTrue(re.search('34.000: StopCondition == true, rel_dist: 30.50 > 30.00, edge: rising', log)  is not None)
        self.assertTrue(re.search('37.900: StopEvent runningState -> endTransition -> standbyState', log)  is not None)

        # Check vehicle key positions
        csv = generate_csv()

        self.assertTrue(re.search('^5.000, 0, Ego, 60.500, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 2.617', csv, re.MULTILINE))
        self.assertTrue(re.search('^5.000, 1, Target, 100.000, -1.535, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('^9.900, 0, Ego, 109.500, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 4.387', csv, re.MULTILINE))
        self.assertTrue(re.search('^9.900, 1, Target, 139.000, -1.535, 0.000, 0.000, 0.000, 0.000, 19.500, 0.000, 4.614', csv, re.MULTILINE))
        self.assertTrue(re.search('^10.000, 0, Ego, 110.500, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 0.961', csv, re.MULTILINE))
        self.assertTrue(re.search('^10.000, 1, Target, 141.000, -1.535, 0.000, 0.000, 0.000, 0.000, 20.000, 0.000, 4.046', csv, re.MULTILINE))
        self.assertTrue(re.search('^10.100, 0, Ego, 111.500, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 3.818', csv, re.MULTILINE))
        self.assertTrue(re.search('^10.100, 1, Target, 142.950, -1.535, 0.000, 0.000, 0.000, 0.000, 19.500, 0.000, 3.334', csv, re.MULTILINE))
        self.assertTrue(re.search('^13.900, 0, Ego, 149.500, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 5.575', csv, re.MULTILINE))
        self.assertTrue(re.search('^13.900, 1, Target, 180.000, -1.535, 0.000, 0.000, 0.000, 0.000, 0.500, 0.000, 2.377', csv, re.MULTILINE))
        self.assertTrue(re.search('^14.000, 0, Ego, 150.500, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 2.149', csv, re.MULTILINE))
        self.assertTrue(re.search('^14.000, 1, Target, 180.000, -1.535, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 2.377', csv, re.MULTILINE))
        self.assertTrue(re.search('^14.100, 0, Ego, 151.500, -1.535, 0.000, 0.000, 0.000, 0.000, 10.000, 0.000, 5.006', csv, re.MULTILINE))
        self.assertTrue(re.search('^14.100, 1, Target, 180.050, -1.535, 0.000, 0.000, 0.000, 0.000, 0.500, 0.000, 2.520', csv, re.MULTILINE))


if __name__ == "__main__":
    # execute only if run as a script
    # unittest.main(argv=['ignored', '-v', 'TestSuite.test_lane_change_at_hw_exit'])

    if len(sys.argv) > 1:
        # Add test case name as argument to run only that test
        # example: smoke_test.py test_follow_ghost
        unittest.main(argv=['ignored', '-v', 'TestSuite.' + sys.argv[1]])
    else:
        unittest.main(verbosity=2)
