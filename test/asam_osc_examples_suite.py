import re
import sys
from test_common import *
import unittest
import argparse
import os.path

COMMON_ESMINI_ARGS = '--headless --fixed_timestep 0.05 --record sim.dat '
OSC_EXAMPLES_PREFIX = './ASAM-OSC-XML-scenarios/ASAM_OpenSCENARIO_v1.4.0_Examples/'


class TestSuite(unittest.TestCase):

    def test_close_vehicle_crossing(self):
        log, _, _, _ = run_scenario(os.path.join(OSC_EXAMPLES_PREFIX + 'CloseVehicleCrossing.xosc'), COMMON_ESMINI_ARGS)

        # Check some scenario events
        self.assertTrue(re.search('Loading .*CloseVehicleCrossing.xosc', log)  is not None)
        self.assertTrue(re.search('.0.350.* Start_of_A2: true, delay: 0.00, 0.3500 > 0.3000, edge: rising', log)  is not None)
        self.assertTrue(re.search('.20.900.* End_of_simulation: true, delay: 0.00, traveled_dist: 320.13 >= 320.00, edge: none', log)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        # Sample when ego passed crossing targets
        self.assertTrue(re.search('\\n17.850, 0, Ego, 26.075, -4.324, 0.000, 0.170, 0.000, 0.000, 6.400, 0.009, 5.958', csv))
        self.assertTrue(re.search('\\n17.850, 1, A1, 23.450, 0.058, 0.000, 4.906, 0.000, 0.000, 6.556, 0.000, 3.768', csv))
        self.assertTrue(re.search('\\n17.850, 2, A2, 22.376, 5.526, 0.000, 4.906, 0.000, 0.000, 6.556, 0.000, 3.849', csv))


    def test_cut_in(self):
        log, _, _, _ = run_scenario(os.path.join(OSC_EXAMPLES_PREFIX + 'CutIn.xosc'), COMMON_ESMINI_ARGS)

        # Check some scenario events
        self.assertTrue(re.search('.43.100.* StartCondition1: true, delay: 0.00, dist: 20.02 > 20.00, edge: rising', log)  is not None)
        self.assertTrue(re.search('.44.900.* Act1 runningState -> endTransition -> completeState', log)  is not None)
        self.assertTrue(re.search('.44.900.* End_of_simulation: true, delay: 10.00, Act1, COMPLETE / END_TRANSITION == COMPLETE, edge: none', log)  is not None)
        self.assertTrue(re.search('.54.900.* End_of_simulation: true', log)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        # Sample before cut-in starts
        self.assertTrue(re.search('\\n43.050, 0, Ego, 2195.843, -7.950, 0.000, 0.000, 0.000, 0.000, 27.778, 0.000, 4.924', csv))
        self.assertTrue(re.search('\\n43.050, 1, A1, 2279.843, -8.450, 0.000, 0.000, 0.000, 0.000, 27.778, 0.000, 4.924', csv))
        self.assertTrue(re.search('\\n43.050, 2, A2, 2215.436, -4.550, 0.000, 0.000, 0.000, 0.000, 30.556, 0.000, 1.043', csv))
        # Sample at start
        self.assertTrue(re.search('\\n43.100, 0, Ego, 2197.232, -7.950, 0.000, 0.000, 0.000, 0.000, 27.778, 0.000, 2.609', csv))
        self.assertTrue(re.search('\\n43.100, 1, A1, 2281.232, -8.450, 0.000, 0.000, 0.000, 0.000, 27.778, 0.000, 2.609', csv))
        self.assertTrue(re.search('\\n43.100, 2, A2, 2216.964, -4.550, 0.000, 0.000, 0.000, 0.000, 30.556, 0.000, 5.408', csv))
        # Sample at complete
        self.assertTrue(re.search('\\n44.900, 0, Ego, 2247.232, -7.950, 0.000, 0.000, 0.000, 0.000, 27.778, 0.000, 0.955', csv))
        self.assertTrue(re.search('\\n44.900, 1, A1, 2331.232, -8.450, 0.000, 0.000, 0.000, 0.000, 27.778, 0.000, 0.955', csv))
        self.assertTrue(re.search('\\n44.900, 2, A2, 2271.795, -8.486, 0.000, 0.000, 0.000, 0.000, 30.556, 0.041, 5.474', csv))

    def test_double_lanechanger(self):
        log, _, _, _ = run_scenario(os.path.join(OSC_EXAMPLES_PREFIX + 'DoubleLaneChanger.xosc'), COMMON_ESMINI_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*DoubleLaneChanger.xosc', log)  is not None)
        self.assertTrue(re.search('.1.650.* MyStartCondition1: true, delay: 0.00, rel_dist: 5.10 > 5.00, edge: rising', log)  is not None)
        self.assertTrue(re.search('.1.650.* MyLaneChangeRightAction initState -> startTransition -> runningState', log)  is not None)
        self.assertTrue(re.search('.3.650.* MyLaneChangeRightAction runningState -> endTransition -> completeState', log)  is not None)
        self.assertTrue(re.search('.3.650.* MyStartCondition2: true, delay: 0.00, MyLaneChangeRightAction, COMPLETE / END_TRANSITION == COMPLETE, edge: rising', log)  is not None)
        self.assertTrue(re.search('.3.650.* MyLaneChangeLeftAction initState -> startTransition -> runningState', log)  is not None)
        self.assertTrue(re.search('.5.650.* MyLaneChangeLeftAction runningState -> endTransition -> completeState', log)  is not None)
        self.assertTrue(re.search('.12.050.* Overtake_finished: true, delay: 0.00, rel_dist: 50.50 > 50.00, edge: rising', log)  is not None)

        # Check vehicle key positions
        csv = generate_csv()
        # Sample before cut-in starts
        self.assertTrue(re.search('\\n1.600, 0, Ego, 170.240, 401.083, 0.000, 1.571, 0.000, 0.000, 36.111, 0.000, 1.717', csv))
        self.assertTrue(re.search('\\n1.600, 1, A1, 166.826, 405.624, 0.000, 1.571, 0.000, 0.000, 47.222, 0.000, 2.245', csv))
        self.assertTrue(re.search('\\n1.600, 2, A2, 170.240, 471.083, 0.000, 1.571, 0.000, 0.000, 36.111, 0.000, 1.717', csv))
        # Sample at start
        self.assertTrue(re.search('\\n1.650, 0, Ego, 170.240, 402.888, 0.000, 1.571, 0.000, 0.000, 36.111, 0.000, 0.592', csv))
        self.assertTrue(re.search('\\n1.650, 1, A1, 166.826, 407.985, 0.000, 1.571, 0.000, 0.000, 47.222, 0.000, 2.708', csv))
        self.assertTrue(re.search('\\n1.650, 2, A2, 170.240, 472.888, 0.000, 1.571, 0.000, 0.000, 36.111, 0.000, 0.592', csv))
        # Sample at first lanechange complete
        self.assertTrue(re.search('\\n3.650, 0, Ego, 170.240, 475.110, 0.000, 1.571, 0.000, 0.000, 36.111, 0.000, 5.879', csv))
        self.assertTrue(re.search('\\n3.650, 1, A1, 170.240, 502.353, 0.000, 1.571, 0.000, 0.000, 47.222, 0.011, 2.372', csv))
        self.assertTrue(re.search('\\n3.650, 2, A2, 170.240, 545.110, 0.000, 1.571, 0.000, 0.000, 36.111, 0.000, 5.879', csv))
        # Sample at second lanechange complete
        self.assertTrue(re.search('\\n5.650, 0, Ego, 170.240, 547.333, 0.000, 1.571, 0.000, 0.000, 36.111, 0.000, 4.883', csv))
        self.assertTrue(re.search('\\n5.650, 1, A1, 166.740, 596.717, 0.000, 1.571, 0.000, 0.000, 47.222, -0.011, 2.036', csv))
        self.assertTrue(re.search('\\n5.650, 2, A2, 170.240, 617.333, 0.000, 1.571, 0.000, 0.000, 36.111, 0.000, 4.883', csv))
        # Sample at overtake complete
        self.assertTrue(re.search('\\n12.050, 0, Ego, 170.240, 778.444, 0.000, 1.571, 0.000, 0.000, 36.111, 0.000, 5.466', csv))
        self.assertTrue(re.search('\\n12.050, 1, A1, 166.740, 898.940, 0.000, 1.571, 0.000, 0.000, 47.222, 0.000, 4.732', csv))
        self.assertTrue(re.search('\\n12.050, 2, A2, 170.240, 848.444, 0.000, 1.571, 0.000, 0.000, 36.111, 0.000, 5.466', csv))


    def test_end_of_traffic_jam_neighboring_lane_occupied(self):
        log, _, _, _ = run_scenario(os.path.join(OSC_EXAMPLES_PREFIX + 'EndofTrafficJamNeighboringLaneOccupied.xosc'), COMMON_ESMINI_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*EndofTrafficJamNeighboringLaneOccupied.xosc', log)  is not None)
        self.assertTrue(re.search('.0.000.* Init EnvironmentAction initState -> startTransition -> runningState', log)  is not None)
        self.assertTrue(re.search('.0.000.* Init EnvironmentAction runningState -> stopTransition -> completeState', log)  is not None)
        self.assertTrue(re.search('.33.000.* StartCondition1: true, delay: 0.00, dist: 54.85 < 55.00, edge: rising', log)  is not None)
        self.assertTrue(re.search('.55.050.* End_of_Simulation: true, delay: 0.00, 55.0500 > 55.0000, edge: none', log)  is not None)

        csv = generate_csv()
        # Sample at ego collision
        self.assertTrue(re.search('\\n44.850, 0, Ego, 1996.657, -11.700, 0.000, 0.000, 0.000, 0.000, 22.222, 0.000, 1.308', csv))
        self.assertTrue(re.search('\\n44.850, 1, A1, 1960.394, -7.500, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.960', csv))
        self.assertTrue(re.search('\\n44.850, 2, A2, 2000.000, -11.200, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\\n44.850, 3, A3, 2000.000, -3.925, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000', csv))

    def test_end_of_traffic_jam(self):
        log, _, _, _ = run_scenario(os.path.join(OSC_EXAMPLES_PREFIX + 'EndOfTrafficJam.xosc'), COMMON_ESMINI_ARGS + "--disable_controllers ")

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*EndOfTrafficJam.xosc', log)  is not None)
        self.assertTrue(re.search('.4.550.* StartCondition1: true, delay: 0.00, traveled_dist: 101.11 >= 100.00, edge: rising', log)  is not None)
        self.assertTrue(re.search('.9.050.* StartCondition1: true, delay: 0.00, traveled_dist: 201.11 >= 200.00, edge: rising', log)  is not None)
        self.assertTrue(re.search('.40.050.* End_of_Simulation: true, delay: 0.00, 40.0500 > 40.0000, edge: none', log)  is not None)

        csv = generate_csv()
        # Sample at ego collision
        self.assertTrue(re.search('\\n25.100, 0, Ego, 1697.228, -8.450, 2.000, 0.000, 6.274, 0.000, 27.778, 0.000, 0.310', csv))
        self.assertTrue(re.search('\\n25.100, 1, A1, 1701.387, -8.450, 2.038, 0.000, 6.274, 0.000, 19.444, 0.000, 6.252', csv))
        self.assertTrue(re.search('\\n25.100, 2, A2, 1713.888, -4.550, 2.152, 0.000, 6.274, 0.000, 19.444, 0.000, 4.270', csv))

    def test_fast_overtake_with_reinitialization(self):
        log, _, _, _ = run_scenario(os.path.join(OSC_EXAMPLES_PREFIX + 'FastOvertakeWithReInitialization.xosc'), COMMON_ESMINI_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*FastOvertakeWithReInitialization.xosc', log)  is not None)
        self.assertTrue(re.search('.5.400.* MyStartCondition1: true, delay: 2.00, MyReduceSpeedAction, RUNNING / START_TRANSITION == START_TRANSITION, edge: rising', log)  is not None)
        self.assertTrue(re.search('.7.400.* SetUpA2 initState -> startTransition -> runningState', log)  is not None)
        self.assertTrue(re.search('.7.400.* Pos\\(166.74, 1451.70, 0.00\\) Rot\\(1.57, 0.00, 0.00\\) roadId 9 laneId -3 s 138.55 offset 0.00 t -7.25', log)  is not None)

        csv = generate_csv()
        # Sample at start
        self.assertTrue(re.search('\\n0.000, 0, Ego, 170.259, 1351.067, 0.000, 1.571, 0.000, 0.000, 36.111, 0.000, 0.000', csv))
        self.assertTrue(re.search('\\n0.000, 1, A1, 170.264, 1470.970, 0.000, 1.571, 0.000, 0.000, 25.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\\n0.000, 2, A2, 148.829, 317.998, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        # Sample at target teleport
        self.assertTrue(re.search('\\n7.400, 0, Ego, 170.259, 1618.289, 0.000, 1.571, 0.000, 0.000, 36.111, 0.000, 3.227', csv))
        self.assertTrue(re.search('\\n7.400, 1, A1, 170.264, 1651.700, 0.000, 1.571, 0.000, 0.000, 20.833, 0.000, 1.148', csv))
        self.assertTrue(re.search('\\n7.400, 2, A2, 166.740, 1451.700, 0.000, 1.571, 0.000, 0.000, 55.556, 0.000, 0.000', csv))


    def test_overtaker(self):
        log, _, _, _ = run_scenario(os.path.join(OSC_EXAMPLES_PREFIX + 'Overtaker.xosc'), COMMON_ESMINI_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*Overtaker.xosc', log)  is not None)
        self.assertTrue(re.search('.8.850.* MyStartCondition1: true, delay: 0.00, rel_dist: 29.88 < 30.00, edge: rising', log)  is not None)
        self.assertTrue(re.search('.8.850.* MyLaneChangeLeftEvent standbyState -> startTransition -> runningState', log)  is not None)
        self.assertTrue(re.search('.13.850.* MyLaneChangeLeftEvent complete after 1 execution', log)  is not None)
        self.assertTrue(re.search('.15.150.* MyStartCondition2: true, delay: 0.00, rel_dist: 5.08 > 5.00, edge: rising', log)  is not None)
        self.assertTrue(re.search('.15.150.* MyLaneChangeRightEvent standbyState -> startTransition -> runningState', log)  is not None)
        self.assertTrue(re.search('.19.650.* End_of_Simulation: true, delay: 10.00, rel_dist: 0.49 > 0.50, edge: falling', log)  is not None)
        self.assertTrue(re.search('.20.150.* MyLaneChangeRightEvent runningState -> endTransition -> completeState', log)  is not None)

        csv = generate_csv()
        # Sample before lanechange start
        self.assertTrue(re.search('\\n8.250, 0, Ego, 170.380, 703.992, 0.000, 1.571, 0.000, 0.000, 36.111, 0.000, 2.960', csv))
        self.assertTrue(re.search('\\n8.250, 1, A1, 170.414, 670.782, 0.000, 1.571, 0.000, 0.000, 41.667, 0.000, 1.966', csv))
        # Sample at lanechange start
        self.assertTrue(re.search('\\n8.850, 0, Ego, 170.380, 725.658, 0.000, 1.571, 0.000, 0.000, 36.111, 0.000, 2.033', csv))
        self.assertTrue(re.search('\\n8.850, 1, A1, 170.414, 695.782, 0.000, 1.571, 0.000, 0.000, 41.667, 0.000, 4.279', csv))
        # Sample at first lanechange finish
        self.assertTrue(re.search('\\n13.850, 0, Ego, 170.380, 906.214, 0.000, 1.571, 0.000, 0.000, 36.111, 0.000, 2.685', csv))
        self.assertTrue(re.search('\\n13.850, 1, A1, 166.740, 904.075, 0.000, 1.571, 0.000, 0.000, 41.667, -0.002, 2.615', csv))
        # Sample at end of simulation condition
        self.assertTrue(re.search('\\n19.650, 0, Ego, 170.380, 1115.658, 0.000, 1.571, 0.000, 0.000, 36.111, 0.000, 4.195', csv))
        self.assertTrue(re.search('\\n19.650, 1, A1, 170.155, 1145.706, 0.000, 1.562, 0.000, 0.000, 41.667, 0.001, 1.941', csv))
        # Sample at second lanechange finish
        self.assertTrue(re.search('\\n20.150, 0, Ego, 170.380, 1133.714, 0.000, 1.571, 0.000, 0.000, 36.111, 0.000, 5.517', csv))
        self.assertTrue(re.search('\\n20.150, 1, A1, 170.240, 1166.539, 0.000, 1.571, 0.000, 0.000, 41.667, 0.002, 4.916', csv))

    def test_sequantal_events_0_100_0_explicit(self):
        log, _, _, _ = run_scenario(os.path.join(OSC_EXAMPLES_PREFIX + 'SequentialEvents_0-100-0kph_Explicit.xosc'), COMMON_ESMINI_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*SequentialEvents_0-100-0kph_Explicit.xosc', log)  is not None)
        self.assertTrue(re.search('.0.050.* AccelerateStartCondition: true, delay: 0.00, 0.0500 > 0.0000, edge: none', log)  is not None)
        self.assertTrue(re.search('.7.000.* BrakeCondition: true, delay: 0.00, AccelerateEvent, COMPLETE / END_TRANSITION == COMPLETE, edge: rising', log)  is not None)

        csv = generate_csv()
        # Sample at start
        self.assertTrue(re.search('\\n0.000, 0, Ego, 20.000, -8.450, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        # Sample before acceleration finish
        self.assertTrue(re.search('\\n6.950, 0, Ego, 115.910, -8.450, 0.000, 0.000, 0.000, 0.000, 27.600, 0.000, 3.852', csv))
        # Sample at acceleration finish
        self.assertTrue(re.search('\\n7.000, 0, Ego, 117.299, -8.450, 0.000, 0.000, 0.000, 0.000, 27.780, 0.000, 1.537', csv))
        # Sample at braking start
        self.assertTrue(re.search('\\n7.050, 0, Ego, 118.678, -8.450, 0.000, 0.000, 0.000, 0.000, 27.580, 0.000, 5.477', csv))
        # Sample at standstill
        self.assertTrue(re.search('\\n13.950, 0, Ego, 213.071, -8.450, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 4.994', csv))

    def test_sequantal_events_0_100_0_implicit(self):
        log, _, _, _ = run_scenario(os.path.join(OSC_EXAMPLES_PREFIX + 'SequentialEvents_0-100-0kph_Implicit.xosc'), COMMON_ESMINI_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*SequentialEvents_0-100-0kph_Implicit.xosc', log)  is not None)
        self.assertTrue(re.search('.0.050.* AccelerateStartCondition: true, delay: 0.00, 0.0500 > 0.0000, edge: none', log)  is not None)
        self.assertTrue(re.search('.7.000.* BrakeStartCondition: true, delay: 0.00, speed: 27.80 >= 27.78, edge: rising', log)  is not None)

        csv = generate_csv()
        # Sample at start
        self.assertTrue(re.search('\\n0.000, 0, Ego, 20.000, -8.450, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        # Sample before acceleration finish
        self.assertTrue(re.search('\\n6.950, 0, Ego, 115.910, -8.450, 0.000, 0.000, 0.000, 0.000, 27.600, 0.000, 3.852', csv))
        # Sample at acceleration finish
        self.assertTrue(re.search('\\n7.000, 0, Ego, 117.300, -8.450, 0.000, 0.000, 0.000, 0.000, 27.800, 0.000, 1.540', csv))
        # Sample at braking start
        self.assertTrue(re.search('\\n7.050, 0, Ego, 118.680, -8.450, 0.000, 0.000, 0.000, 0.000, 27.600, 0.000, 5.483', csv))
        # Sample at standstill
        self.assertTrue(re.search('\\n13.950, 0, Ego, 213.210, -8.450, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 5.391', csv))

    def test_simple_overtake(self):
        log, _, _, _ = run_scenario(os.path.join(OSC_EXAMPLES_PREFIX + 'SimpleOvertake.xosc'), COMMON_ESMINI_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*SimpleOvertake.xosc', log)  is not None)
        self.assertTrue(re.search('.5.050.* Relative Distance: true, delay: 0.00, rel_dist: 29.94 < 30.00, edge: rising', log)  is not None)
        self.assertTrue(re.search('.12.250.* Relative Distance: true, delay: 0.00, rel_dist: 10.01 > 10.00, edge: rising', log)  is not None)
        self.assertTrue(re.search('.12.250.* Simulation Time: true, delay: 0.00, 12.2500 > 5.0000, edge: none', log)  is not None)
        self.assertTrue(re.search('.20.050.* Time limit: true, delay: 0.00, 20.0500 > 20.0000, edge: rising', log)  is not None)

        csv = generate_csv()
        # Sample before overtake start
        self.assertTrue(re.search('\\n5.000, 0, Vehicle 1, 170.490, 523.482, 0.000, 1.571, 0.000, 0.000, 41.667, 0.000, 4.619', csv))
        self.assertTrue(re.search('\\n5.000, 1, Vehicle 2, 170.490, 553.704, 0.000, 1.571, 0.000, 0.000, 36.111, 0.000, 0.652', csv))
        # Sample at overtake start
        self.assertTrue(re.search('\\n5.050, 0, Vehicle 1, 170.490, 525.566, 0.000, 1.571, 0.000, 0.000, 41.667, 0.000, 4.288', csv))
        self.assertTrue(re.search('\\n5.050, 1, Vehicle 2, 170.490, 555.510, 0.000, 1.571, 0.000, 0.000, 36.111, 0.000, 5.811', csv))
        # Sample at overtake finish
        self.assertTrue(re.search('\\n9.850, 0, Vehicle 1, 166.717, 725.523, 0.000, 1.571, 0.000, 0.000, 41.667, -0.003, 3.947', csv))
        self.assertTrue(re.search('\\n9.850, 1, Vehicle 2, 170.490, 728.843, 0.000, 1.571, 0.000, 0.000, 36.111, 0.000, 4.677', csv))
        # Sample before second overtake start
        self.assertTrue(re.search('\\n12.000, 0, Vehicle 1, 166.717, 815.106, 0.000, 1.571, 0.000, 0.000, 41.667, 0.000, 2.288', csv))
        self.assertTrue(re.search('\\n12.000, 1, Vehicle 2, 170.490, 806.482, 0.000, 1.571, 0.000, 0.000, 36.111, 0.000, 0.308', csv))
        # Sample at second overtake start
        self.assertTrue(re.search('\\n12.250, 0, Vehicle 1, 166.717, 825.523, 0.000, 1.571, 0.000, 0.000, 41.667, 0.000, 0.634', csv))
        self.assertTrue(re.search('\\n12.250, 1, Vehicle 2, 170.490, 815.510, 0.000, 1.571, 0.000, 0.000, 36.111, 0.000, 0.969', csv))
        # Sample at second overtake finish
        self.assertTrue(re.search('\\n16.250, 0, Vehicle 1, 170.240, 992.144, 0.000, 1.571, 0.000, 0.000, 41.667, 0.004, 5.586', csv))
        self.assertTrue(re.search('\\n16.250, 1, Vehicle 2, 170.490, 959.954, 0.000, 1.571, 0.000, 0.000, 36.111, 0.000, 5.260', csv))

    def test_slow_preceding_vehicle_determenistic_parameter_set(self):
        # Tests so running parameter distribution scenario with same road re-uses the road

        scenario = os.path.join(OSC_EXAMPLES_PREFIX + 'SlowPrecedingVehicleDeterministicParameterSet.xosc')

        _, _, _, output = run_scenario(scenario, '--return_nr_permutations', ignoreReturnCode=True, read_log=False)
        match = re.search(r'Nr permutations: (\d+)', output)
        self.assertTrue(match is not None, output)
        permutations = int(match.group(1))

        self.assertGreater(permutations, 1)

        # Remove any logfiles that might exist prior to execution
        for i in range(1, permutations + 1):
            for filename in (f"log_{i}_of_{permutations}.txt", f"sim_{i}_of_{permutations}.dat"):
                if os.path.isfile(filename):
                    os.remove(filename)

        _, _, _, _ = run_scenario(scenario, COMMON_ESMINI_ARGS, read_log=False)

        for i in range(1, permutations + 1):
            with open(f"log_{i}_of_{permutations}.txt", 'r') as logfile:
                log = logfile.read()
            self.assertTrue(re.search(r'Loading .*SlowPrecedingVehicle\.xosc', log) is not None)
            self.assertTrue(re.search(rf'Parameter permutation {i}/{permutations}\b', log) is not None)
            if i == 1:
                self.assertTrue(re.search(r'Loaded OpenDRIVE: ASAM-OSC-XML-scenarios/ASAM_OpenSCENARIO_v1\.4\.0_Examples/Databases/AB_RQ31_R2000\.xodr', log) is not None)
            else:
                self.assertTrue(re.search(r'Reusing OpenDRIVE: ASAM-OSC-XML-scenarios/ASAM_OpenSCENARIO_v1\.4\.0_Examples/Databases/AB_RQ31_R2000\.xodr', log) is not None)
            # Remove the files to keep clutter down, if debugging comment away these two lines
            os.remove(f"log_{i}_of_{permutations}.txt")
            os.remove(f"sim_{i}_of_{permutations}.dat")

    def test_slow_preceding_vehicle(self):
        log, _, _, _ = run_scenario(os.path.join(OSC_EXAMPLES_PREFIX + 'SlowPrecedingVehicle.xosc'), COMMON_ESMINI_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*SlowPrecedingVehicle.xosc', log)  is not None)
        self.assertTrue(re.search('.34.750.* End_of_simulation_based_on_distance: true, delay: 0.00, TTC: 0.46 < 0.50, edge none', log)  is not None)

        csv = generate_csv()
        # Sample at simulation end
        self.assertTrue(re.search('\\n34.750, 0, Ego, 960.099, 65.988, 0.000, 0.232, 0.000, 0.000, 27.778, 0.001, 5.923', csv))
        self.assertTrue(re.search('\\n34.750, 1, A1, 966.921, 67.612, 0.000, 0.235, 0.000, 0.000, 22.222, 0.001, 0.929', csv))

    def test_synchronized_arrival_to_intersection(self):
        log, _, _, _ = run_scenario(os.path.join(OSC_EXAMPLES_PREFIX + 'SynchronizedArrivalToIntersection.xosc'), COMMON_ESMINI_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*SynchronizedArrivalToIntersection.xosc', log)  is not None)
        self.assertTrue(re.search('.0.000.* Pos\\(42.98, -71.25, 0.00\\) Rot\\(1.78, 0.00, 0.00\\) roadId 0 laneId 1 s 63.00 offset 0.00 t 1.75', log)  is not None)
        self.assertTrue(re.search('.0.000.* Pos\\(-15.70, -10.55, 0.00\\) Rot\\(0.15, 0.00, 0.00\\) roadId 3 laneId -1 s 80.00 offset 0.00 t -1.75', log)  is not None)
        self.assertTrue(re.search('.1.150.* SyncConditionNPC: true, delay: 0.00, dist: 74.90 < 75.00, edge: rising', log)  is not None)
        self.assertTrue(re.search('.4.350.* Entering Stead State according to criteria but not enough time to reach destination', log)  is not None)
        self.assertTrue(re.search('.5.750.* Synchronize masterTimeToDest \\(0.003\\) reached within this timestep \\(0.050\\)', log)  is not None)
        self.assertTrue(re.search('.6.500.* End_of_simulation: true, delay: 0.00, 1 collision\\(s\\): Ego and NPC, edge: none', log)  is not None)

        csv = generate_csv()
        # Sample at synchronize trigger
        self.assertTrue(re.search('\\n1.150, 0, Ego, 40.546, -60.011, 0.000, 1.792, 0.000, 0.000, 10.000, 0.003, 1.441', csv))
        self.assertTrue(re.search('\\n1.150, 1, NPC, -15.703, -10.552, 0.000, 0.146, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        # Sample at steady state reached
        self.assertTrue(re.search('\\n4.300, 0, Ego, 33.378, -29.338, 0.000, 1.801, 0.000, 0.000, 10.000, -0.001, 3.477', csv))
        self.assertTrue(re.search('\\n4.300, 1, NPC, 8.204, -7.044, 0.000, 0.146, 0.000, 0.000, 6.865, 0.000, 6.206', csv))
        # Sample at synchronize complete
        self.assertTrue(re.search('\\n5.750, 0, Ego, 30.135, -15.206, 0.000, 1.790, 0.000, 0.000, 10.000, -0.003, 0.923', csv))
        self.assertTrue(re.search('\\n5.750, 1, NPC, 18.247, -5.570, 0.000, 0.146, 0.000, 0.000, 7.000, 0.001, 3.790', csv))
        # Sample at collision
        self.assertTrue(re.search('\\n6.500, 0, Ego, 28.542, -7.876, 0.000, 1.779, 0.000, 0.000, 10.000, -0.006, 3.502', csv))
        self.assertTrue(re.search('\\n6.500, 1, NPC, 23.435, -4.766, 0.000, 0.162, 0.000, 0.000, 7.000, 0.009, 6.224', csv))

    def test_traffic_jam(self):
        log, _, _, _ = run_scenario(os.path.join(OSC_EXAMPLES_PREFIX + 'TrafficJam.xosc'), COMMON_ESMINI_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*TrafficJam.xosc', log)  is not None)
        self.assertTrue(re.search('.0.000.* Pos\\(163.40, 350.80, 0.00\\) Rot\\(1.57, 0.00, 0.00\\) roadId 3 laneId -2 s 37.65 offset -0.16 t -3.91', log)  is not None)
        self.assertTrue(re.search('.0.000.* Pos\\(170.84, 495.80, 0.00\\) Rot\\(1.57, 0.00, 0.00\\) roadId 3 laneId -4 s 182.65 offset -0.60 t -11.35', log)  is not None)
        self.assertTrue(re.search('.0.000.* Pos\\(170.84, 503.80, 0.00\\) Rot\\(1.57, 0.00, 0.00\\) roadId 3 laneId -4 s 190.65 offset -0.60 t -11.35', log)  is not None)
        self.assertTrue(re.search('.0.000.* Pos\\(168.34, 498.80, 0.00\\) Rot\\(1.57, 0.00, 0.00\\) roadId 3 laneId -3 s 185.65 offset -1.60 t -8.85', log)  is not None)
        self.assertTrue(re.search('.0.000.* Pos\\(168.34, 506.80, 0.00\\) Rot\\(1.57, 0.00, 0.00\\) roadId 3 laneId -3 s 193.65 offset -1.60 t -8.85', log)  is not None)
        self.assertTrue(re.search('.0.000.* Pos\\(162.50, 501.80, 0.00\\) Rot\\(1.57, 0.00, 0.00\\) roadId 3 laneId -2 s 188.65 offset 0.74 t -3.01', log)  is not None)
        self.assertTrue(re.search('.0.000.* Pos\\(162.50, 509.80, 0.00\\) Rot\\(1.57, 0.00, 0.00\\) roadId 3 laneId -2 s 196.65 offset 0.74 t -3.01', log)  is not None)

        csv = generate_csv()
        # Sample at init
        self.assertTrue(re.search('\\n0.000, 0, Ego, 163.400, 350.800, 0.000, 1.571, 0.000, 0.000, 36.100, 0.000, 0.000', csv))
        self.assertTrue(re.search('\\n0.000, 1, A1, 170.840, 495.800, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\\n0.000, 2, A2, 170.840, 503.800, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\\n0.000, 3, A3, 168.340, 498.800, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\\n0.000, 4, A4, 168.340, 506.800, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\\n0.000, 5, A5, 162.500, 501.800, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\\n0.000, 6, A6, 162.500, 509.800, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        # Sample at collision
        self.assertTrue(re.search('\\n4.050, 0, Ego, 163.400, 497.005, 0.000, 1.571, 0.000, 0.000, 36.100, 0.000, 3.038', csv))
        self.assertTrue(re.search('\\n4.050, 1, A1, 170.840, 495.800, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\\n4.050, 2, A2, 170.840, 503.800, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\\n4.050, 3, A3, 168.340, 498.800, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\\n4.050, 4, A4, 168.340, 506.800, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\\n4.050, 5, A5, 162.500, 501.800, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\\n4.050, 6, A6, 162.500, 509.800, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv))

    def test_trailer_connect(self):
        log, _, _, _ = run_scenario(os.path.join(OSC_EXAMPLES_PREFIX + 'TrailerConnect.xosc'), COMMON_ESMINI_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('Loading .*TrailerConnect.xosc', log)  is not None)
        self.assertTrue(re.search('.2.000.* ReverseTrigger: true, delay: 0.00, 2.0000 >= 1.0000, edge: none', log)  is not None)
        self.assertTrue(re.search('.8.750.* ConnectEvent complete after 1 execution', log)  is not None)
        self.assertTrue(re.search('.10.750.* DisconnectEventCondition: true, delay: 4.00, ForwardEvent, RUNNING / START_TRANSITION == START_TRANSITION, edge: rising', log)  is not None)
        self.assertTrue(re.search('.14.750.* DisconnectEventCondition: true\\n', log)  is not None)

        # Check vehicle key positions
        csv = generate_csv()

        # Sample at start
        self.assertTrue(re.search('\\n0.000, 0, Trailer, 1.700, 13.500, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('\\n0.000, 1, Car, -10.300, 30.200, 0.000, 3.142, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        # Sample at connect event
        self.assertTrue(re.search('\\n8.750, 0, Trailer, 1.700, 13.500, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('\\n8.750, 1, Car, 1.698, 18.047, 0.000, 1.571, 0.000, 0.000, 0.000, 0.061, -4.020', csv, re.MULTILINE))
        # Sample at disconnect event condition
        self.assertTrue(re.search('\\n10.750, 0, Trailer, 1.700, 13.597, 0.000, 1.572, 0.000, 0.000, 0.000, 0.000, 0.000', csv, re.MULTILINE))
        self.assertTrue(re.search('\\n10.750, 1, Car, 1.698, 18.047, 0.000, 1.571, 0.000, 0.000, 4.000, 0.061, -4.020', csv, re.MULTILINE))
        # Sample at disconnect
        self.assertTrue(re.search('\\n14.750, 0, Trailer, 6.860, 27.770, 0.000, 0.685, 0.000, 0.000, 4.000, 0.000, 1.732', csv, re.MULTILINE))
        self.assertTrue(re.search('\\n14.750, 1, Car, 10.499, 30.253, 0.000, 0.365, 0.000, 0.000, 4.000, -0.198, 3.995', csv, re.MULTILINE))


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
