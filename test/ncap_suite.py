import re
from test_common import *
import unittest
import argparse

# run all scenarios from https://github.com/vectorgrp/OSC-NCAP-scenarios
# check some key events and positions

ESMINI_PATH = '../'
COMMON_ARGS = '--headless --fixed_timestep 0.05 --record sim.dat --align_routepositions '
NCAP_PREFIX = './OSC-NCAP-scenarios/OpenSCENARIO/NCAP/'


class TestSuite(unittest.TestCase):

    def test_AEB_C2C_2023_NCAP_AEB_C2C_CCR_2023(self):

        log, duration, cpu_time, _ = run_scenario(os.path.join(NCAP_PREFIX + 'AEB_C2C_2023/NCAP_AEB_C2C_CCR_2023.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*NCAP_AEB_C2C_CCR_2023', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('\n.4.250.* DetectCollision: true, delay: 0.00, 1 collision\\(s\\): Ego and GVT, edge: none', log)  is not None)
        self.assertTrue(re.search('\n.4.250.* AtCollision standbyState -> startTransition -> runningState', log)  is not None)
        self.assertTrue(re.search('\n.4.250.* Set_Variables complete after 1 execution', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n4.200, 0, Ego, 73.333, -14.000, 0.000, 0.000, 0.000, 0.000, 5.556, 0.000, 3.835', csv))
        self.assertTrue(re.search('\n4.200, 1, GVT, 77.778, -14.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n4.250, 0, Ego, 73.611, -14.000, 0.000, 0.000, 0.000, 0.000, 5.556, 0.000, 4.628', csv))
        self.assertTrue(re.search('\n4.250, 1, GVT, 77.778, -14.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000', csv))

    def test_AEB_C2C_2023_NCAP_AEB_C2C_CCFhol_2023(self):

        log, duration, cpu_time, _ = run_scenario(os.path.join(NCAP_PREFIX + 'AEB_C2C_2023/NCAP_AEB_C2C_CCFhol_2023.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*NCAP_AEB_C2C_CCFhol_2023', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('\n.7.850.* GVT_LaneChange initState -> startTransition -> runningState', log)  is not None)
        self.assertTrue(re.search('\n.12.350.* DetectCollision: true, delay: 0.00, 1 collision\\(s\\): Ego and GVT, edge: none', log)  is not None)
        self.assertTrue(re.search('\n.13.400.* storyBoard runningState -> stopTransition -> completeState', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n7.850, 0, Ego, 202.639, -1.750, 0.000, 0.000, 0.000, 0.000, 19.444, 0.000, 2.571', csv))
        self.assertTrue(re.search('\n7.850, 1, GVT, 383.090, 1.750, 0.000, 3.142, 0.000, 0.000, 19.444, 0.000, 0.083', csv))
        self.assertTrue(re.search('\n7.850, 2, SOV, 359.540, 1.750, 0.000, 3.142, 0.000, 0.000, 19.444, 0.000, 3.588', csv))
        self.assertTrue(re.search('\n7.900, 0, Ego, 203.611, -1.750, 0.000, 0.000, 0.000, 0.000, 19.444, 0.000, 5.349', csv))
        self.assertTrue(re.search('\n7.900, 1, GVT, 382.118, 1.748, 0.000, 3.145, 0.000, 0.000, 19.444, 0.013, 2.860', csv))
        self.assertTrue(re.search('\n7.900, 2, SOV, 358.568, 1.750, 0.000, 3.142, 0.000, 0.000, 19.444, 0.000, 0.083', csv))
        self.assertTrue(re.search('\n9.250, 0, Ego, 229.861, -1.750, 0.000, 0.000, 0.000, 0.000, 19.444, 0.000, 4.951', csv))
        self.assertTrue(re.search('\n9.250, 1, GVT, 355.920, 0.287, 0.000, 3.239, 0.000, 0.000, 19.444, 0.000, 2.462', csv))
        self.assertTrue(re.search('\n9.250, 2, SOV, 332.318, 1.750, 0.000, 3.142, 0.000, 0.000, 19.444, 0.000, 5.968', csv))
        self.assertTrue(re.search('\n12.250, 0, Ego, 288.194, -1.750, 0.000, 0.000, 0.000, 0.000, 19.444, 0.000, 1.972', csv))
        self.assertTrue(re.search('\n12.250, 1, GVT, 297.664, -1.713, 0.000, 3.142, 0.000, 0.000, 19.444, 0.000, 5.766', csv))
        self.assertTrue(re.search('\n12.250, 2, SOV, 273.985, 1.750, 0.000, 3.142, 0.000, 0.000, 19.444, 0.000, 2.988', csv))
        self.assertTrue(re.search('\n12.300, 0, Ego, 289.167, -1.750, 0.000, 0.000, 0.000, 0.000, 19.444, 0.000, 4.749', csv))
        self.assertTrue(re.search('\n12.300, 1, GVT, 296.692, -1.713, 0.000, 3.142, 0.000, 0.000, 19.444, 0.000, 2.261', csv))
        self.assertTrue(re.search('\n12.300, 2, SOV, 273.013, 1.750, 0.000, 3.142, 0.000, 0.000, 19.444, 0.000, 5.766', csv))

        self.assertTrue(re.search('\n', csv))
        self.assertTrue(re.search('\n', csv))
        self.assertTrue(re.search('\n', csv))
        self.assertTrue(re.search('\n', csv))
        self.assertTrue(re.search('\n', csv))
        self.assertTrue(re.search('\n', csv))
        self.assertTrue(re.search('\n', csv))
        self.assertTrue(re.search('\n', csv))
        self.assertTrue(re.search('\n', csv))
        self.assertTrue(re.search('\n', csv))
        self.assertTrue(re.search('\n', csv))
        self.assertTrue(re.search('\n', csv))

    def test_AEB_C2C_2023_NCAP_AEB_C2C_CCFhos_2023(self):

        log, duration, cpu_time, _ = run_scenario(os.path.join(NCAP_PREFIX + 'AEB_C2C_2023/NCAP_AEB_C2C_CCFhos_2023.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*NCAP_AEB_C2C_CCFhos_2023', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('\n.7.850.* DetectCollision: true, delay: 0.00, 1 collision\\(s\\): Ego and GVT, edge: none', log)  is not None)
        self.assertTrue(re.search('\n.8.900.* storyBoard runningState -> stopTransition -> completeState', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n7.800, 0, Ego, 201.667, -1.750, 0.000, 0.000, 0.000, 0.000, 19.444, 0.000, 6.077', csv))
        self.assertTrue(re.search('\n7.800, 1, GVT, 209.444, -1.750, 0.000, 3.142, 0.000, 0.000, 19.444, 0.000, 6.077', csv))
        self.assertTrue(re.search('\n7.850, 0, Ego, 202.639, -1.750, 0.000, 0.000, 0.000, 0.000, 19.444, 0.000, 2.571', csv))
        self.assertTrue(re.search('\n7.850, 1, GVT, 208.472, -1.750, 0.000, 3.142, 0.000, 0.000, 19.444, 0.000, 2.571', csv))

    def test_AEB_C2C_2023_NCAP_AEB_C2C_CCFtap_2023(self):

        log, duration, cpu_time, _ = run_scenario(os.path.join(NCAP_PREFIX + 'AEB_C2C_2023/NCAP_AEB_C2C_CCFtap_2023.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*NCAP_AEB_C2C_CCFtap_2023', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('\n.24.150.* Synchronize masterTimeToDest \\(0.002\\) reached within this timestep \\(0.050\\)', log)  is not None)
        self.assertTrue(re.search('\n.24.300.* DetectCollision: true, delay: 0.00, 1 collision\\(s\\): Ego and Target, edge: none', log)  is not None)
        self.assertTrue(re.search('\n.27.350.* storyBoard runningState -> stopTransition -> completeState', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n24.250, 0, Ego, 157.264, -0.920, 0.000, 0.375, 0.000, 0.000, 2.778, 0.348, 3.965', csv))
        self.assertTrue(re.search('\n24.250, 1, Target, 163.965, 1.750, 0.000, 3.142, 0.000, 0.000, 8.333, 0.000, 4.844', csv))
        self.assertTrue(re.search('\n24.300, 0, Ego, 157.392, -0.868, 0.000, 0.390, 0.000, 0.000, 2.778, 0.348, 4.362', csv))
        self.assertTrue(re.search('\n24.300, 1, Target, 163.548, 1.750, 0.000, 3.142, 0.000, 0.000, 8.333, 0.000, 6.034', csv))
        self.assertTrue(re.search('\n27.350, 0, Ego, 162.721, 5.309, 0.000, 1.321, 0.000, 0.000, 2.778, 0.316, 3.435', csv))
        self.assertTrue(re.search('\n27.350, 1, Target, 138.131, 1.750, 0.000, 3.142, 0.000, 0.000, 8.333, 0.000, 3.255', csv))

    def test_AEB_VRU_2023_NCAP_AEB_VRU_CBFA_2023(self):

        log, duration, cpu_time, _ = run_scenario(os.path.join(NCAP_PREFIX + 'AEB_VRU_2023/NCAP_AEB_VRU_CBFA_2023.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*NCAP_AEB_VRU_CBFA_2023', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('\n.0.000.* EgoSpeedReached: true, delay: 0.00, speed: 8.33 > 8.17, edge: none', log)  is not None)
        self.assertTrue(re.search('\n.6.750.* DetectCollision: true, delay: 0.00, 1 collision\\(s\\): Ego and VRU, edge: none', log)  is not None)
        self.assertTrue(re.search('\n.7.800.* storyBoard runningState -> stopTransition -> completeState', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n6.700, 0, Ego, 155.833, -1.750, 0.000, 0.000, 0.000, 0.000, 8.333, 0.000, 2.444', csv))
        self.assertTrue(re.search('\n6.700, 1, VRU, 159.750, -1.043, 0.000, 4.712, 0.000, 0.000, 5.556, 0.000, 5.438', csv))
        self.assertTrue(re.search('\n6.700, 3, ObstructionLarge, 157.590, 21.060, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n6.700, 2, ObstructionSmall, 157.575, 16.442, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n6.750, 0, Ego, 156.250, -1.750, 0.000, 0.000, 0.000, 0.000, 8.333, 0.000, 3.635', csv))
        self.assertTrue(re.search('\n6.750, 1, VRU, 159.750, -1.320, 0.000, 4.712, 0.000, 0.000, 5.556, 0.000, 6.231', csv))
        self.assertTrue(re.search('\n6.750, 3, ObstructionLarge, 157.590, 21.060, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n6.750, 2, ObstructionSmall, 157.575, 16.442, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv))


    def test_AEB_VRU_2023_NCAP_AEB_VRU_CBLA_2023(self):

        log, duration, cpu_time, _ = run_scenario(os.path.join(NCAP_PREFIX + 'AEB_VRU_2023/NCAP_AEB_VRU_CBLA_2023.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*NCAP_AEB_VRU_CBLA_2023', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('\n.5.450.* AtEgoApproaching: true, delay: 0.00, rel_dist: 26.78 <= 26.83, edge: rising', log)  is not None)
        self.assertTrue(re.search('\n.13.600.* DetectCollision: true, delay: 0.00, 1 collision\\(s\\): Ego and VRU, edge: none', log)  is not None)
        self.assertTrue(re.search('\n.14.650.* StopAfterCollision: true\\n', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n5.450, 0, Ego, 87.847, -14.000, 0.000, 0.000, 0.000, 0.000, 6.944, 0.000, 1.321', csv))
        self.assertTrue(re.search('\n5.450, 1, VRU, 118.500, -14.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n5.500, 0, Ego, 88.194, -14.000, 0.000, 0.000, 0.000, 0.000, 6.944, 0.000, 2.313', csv))
        self.assertTrue(re.search('\n5.500, 1, VRU, 118.505, -14.000, 0.000, 0.000, 0.000, 0.000, 0.100, 0.000, 0.014', csv))
        self.assertTrue(re.search('\n13.550, 0, Ego, 144.097, -14.000, 0.000, 0.000, 0.000, 0.000, 6.944, 0.000, 4.955', csv))
        self.assertTrue(re.search('\n13.550, 1, VRU, 148.013, -14.000, 0.000, 0.000, 0.000, 0.000, 4.167, 0.000, 2.642', csv))
        self.assertTrue(re.search('\n13.600, 0, Ego, 144.444, -14.000, 0.000, 0.000, 0.000, 0.000, 6.944, 0.000, 5.947', csv))
        self.assertTrue(re.search('\n13.600, 1, VRU, 148.222, -14.000, 0.000, 0.000, 0.000, 0.000, 4.167, 0.000, 3.238', csv))

    def test_AEB_VRU_2023_NCAP_AEB_VRU_CBNAO_2023(self):

        log, duration, cpu_time, _ = run_scenario(os.path.join(NCAP_PREFIX + 'AEB_VRU_2023/NCAP_AEB_VRU_CBNAO_2023.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*NCAP_AEB_VRU_CBNAO_2023', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('\n.0.000.* VRU_SynchronizeEvent standbyState -> startTransition -> runningState', log)  is not None)
        self.assertTrue(re.search('\n.9.400.* Synchronize masterTimeToDest \\(0.010\\) reached within this timestep \\(0.050\\)', log)  is not None)
        self.assertTrue(re.search('\n.9.450.* DetectCollision: true, delay: 0.00, 1 collision\\(s\\): Ego and VRU, edge: none', log)  is not None)
        self.assertTrue(re.search('\n.10.500.* StopAfterCollision: true\\n', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n9.400, 0, Ego, 159.444, -1.750, 0.000, 0.000, 0.000, 0.000, 2.778, 0.000, 5.488', csv))
        self.assertTrue(re.search('\n9.400, 1, VRU, 163.250, -2.397, 0.000, 1.571, 0.000, 0.000, 2.778, 0.000, 5.875', csv))
        self.assertTrue(re.search('\n9.400, 3, ObstructionLarge, 157.290, -6.110, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n9.400, 2, ObstructionSmall, 157.305, -10.728, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n9.450, 0, Ego, 159.583, -1.750, 0.000, 0.000, 0.000, 0.000, 2.778, 0.000, 5.885', csv))
        self.assertTrue(re.search('\n9.450, 1, VRU, 163.250, -2.258, 0.000, 1.571, 0.000, 0.000, 2.778, 0.000, 6.272', csv))
        self.assertTrue(re.search('\n9.450, 3, ObstructionLarge, 157.290, -6.110, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n9.450, 2, ObstructionSmall, 157.305, -10.728, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv))

    def test_AEB_VRU_2023_NCAP_AEB_VRU_CBNA_2023(self):

        log, duration, cpu_time, _ = run_scenario(os.path.join(NCAP_PREFIX + 'AEB_VRU_2023/NCAP_AEB_VRU_CBNA_2023.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*NCAP_AEB_VRU_CBNA_2023', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('\n.6.950.* Synchronize masterTimeToDest \\(0.026\\) reached within this timestep \\(0.050\\)', log)  is not None)
        self.assertTrue(re.search('\n.7.000.* DetectCollision: true, delay: 0.00, 1 collision\\(s\\): Ego and VRU, edge: none', log)  is not None)
        self.assertTrue(re.search('\n.8.050.* storyBoard runningState -> stopTransition -> completeState', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n6.950, 0, Ego, 157.917, -1.750, 0.000, 0.000, 0.000, 0.000, 8.333, 0.000, 2.113', csv))
        self.assertTrue(re.search('\n6.950, 1, VRU, 161.910, -2.478, 0.000, 1.571, 0.000, 0.000, 4.167, 0.000, 3.661', csv))
        self.assertTrue(re.search('\n6.950, 3, ObstructionLarge, 159.750, -19.560, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n6.950, 2, ObstructionSmall, 159.765, -24.178, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n7.000, 0, Ego, 158.333, -1.750, 0.000, 0.000, 0.000, 0.000, 8.333, 0.000, 3.304', csv))
        self.assertTrue(re.search('\n7.000, 1, VRU, 161.910, -2.269, 0.000, 1.571, 0.000, 0.000, 4.167, 0.000, 4.256', csv))
        self.assertTrue(re.search('\n7.000, 3, ObstructionLarge, 159.750, -19.560, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n7.000, 2, ObstructionSmall, 159.765, -24.178, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv))

    def test_AEB_VRU_2023_NCAP_AEB_VRU_CPNA_2023(self):

        log, duration, cpu_time, _ = run_scenario(os.path.join(NCAP_PREFIX + 'AEB_VRU_2023/NCAP_AEB_VRU_CPNA_2023.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*NCAP_AEB_VRU_CPNA_2023', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('\n.5.500.* Synchronize masterTimeToDest \\(0.047\\) reached within this timestep \\(0.050\\)', log)  is not None)
        self.assertTrue(re.search('\n.5.550.* DetectCollision: true, delay: 0.00, 1 collision\\(s\\): Ego and VRU, edge: none', log)  is not None)
        self.assertTrue(re.search('\n.5.600.* StopAfterCollision: true, delay: 1.00, variable collisionDetected true == true, edge: none', log)  is not None)
        self.assertTrue(re.search('\n.6.600.* StopAfterCollision: true', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n5.500, 0, Ego, 95.833, -14.000, 0.000, 0.000, 0.000, 0.000, 8.333, 0.000, 5.289', csv))
        self.assertTrue(re.search('\n5.500, 1, VRU, 100.000, -14.527, 0.000, 1.571, 0.000, 0.000, 1.389, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n5.550, 0, Ego, 96.250, -14.000, 0.000, 0.000, 0.000, 0.000, 8.333, 0.000, 0.196', csv))
        self.assertTrue(re.search('\n5.550, 1, VRU, 100.000, -14.458, 0.000, 1.571, 0.000, 0.000, 1.389, 0.000, 0.000', csv))

    def test_AEB_VRU_2023_NCAP_AEB_VRU_CPNCO_2023(self):

        log, duration, cpu_time, _ = run_scenario(os.path.join(NCAP_PREFIX + 'AEB_VRU_2023/NCAP_AEB_VRU_CPNCO_2023.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*NCAP_AEB_VRU_CPNCO_2023', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('\n.3.400.* Entering Stead State according to criteria but not enough time to reach destination', log)  is not None)
        self.assertTrue(re.search('\n.5.600.* DetectCollision: true, delay: 0.00, 1 collision\\(s\\): Ego and VRU, edge: none', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n5.550, 0, Ego, 96.250, -14.000, 0.000, 0.000, 0.000, 0.000, 8.333, 0.000, 0.196', csv))
        self.assertTrue(re.search('\n5.550, 1, VRU, 100.000, -14.041, 0.000, 1.571, 0.000, 0.000, 1.389, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n5.550, 2, ObstructionSmall, 95.325, -16.817, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n5.550, 3, ObstructionLarge, 89.927, -16.817, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n5.600, 0, Ego, 96.667, -14.000, 0.000, 0.000, 0.000, 0.000, 8.333, 0.000, 1.386', csv))
        self.assertTrue(re.search('\n5.600, 1, VRU, 100.000, -13.972, 0.000, 1.571, 0.000, 0.000, 1.389, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n5.600, 2, ObstructionSmall, 95.325, -16.817, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n5.600, 3, ObstructionLarge, 89.927, -16.817, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000', csv))

    def test_AEB_VRU_2023_NCAP_AEB_VRU_CPTA_2023(self):

        log, duration, cpu_time, _ = run_scenario(os.path.join(NCAP_PREFIX + 'AEB_VRU_2023/NCAP_AEB_VRU_CPTA_2023.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*NCAP_AEB_VRU_CPTA_2023', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('\n.21.350.* Entering Stead State according to criteria but not enough time to reach destination', log)  is not None)
        self.assertTrue(re.search('\n.27.500.* DetectCollision: true, delay: 0.00, 1 collision\\(s\\): Ego and VRU, edge: none', log)  is not None)
        self.assertTrue(re.search('\n.27.500.* Synchronization_CollisionDetection runningState -> endTransition -> completeState', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n27.450, 0, Ego, 162.780, 5.580, 0.000, 1.345, 0.000, 0.000, 2.778, 0.266, 4.229', csv))
        self.assertTrue(re.search('\n27.450, 1, VRU, 163.241, 9.500, 0.000, 6.283, 0.000, 0.000, 1.389, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n27.500, 0, Ego, 162.808, 5.716, 0.000, 1.356, 0.000, 0.000, 2.778, 0.262, 4.626', csv))
        self.assertTrue(re.search('\n27.500, 1, VRU, 163.310, 9.500, 0.000, 6.283, 0.000, 0.000, 1.389, 0.000, 0.000', csv))

    def test_AEB_VRU_2023_NCAP_AEB_VRU_CPRA_Cm_2023(self):

        log, duration, cpu_time, _ = run_scenario(os.path.join(NCAP_PREFIX + 'AEB_VRU_2023/NCAP_AEB_VRU_CPRA_Cm_2023.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*NCAP_AEB_VRU_CPRA_Cm_2023', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('\n.6.450.* Entering Stead State according to criteria but not enough time to reach destination', log)  is not None)
        self.assertTrue(re.search('\n.8.300.* DetectCollision: true, delay: 0.00, 1 collision\\(s\\): Ego and VRU, edge: none', log)  is not None)
        self.assertTrue(re.search('\n.8.350.* StopAfterCollision: true, delay: 1.00, variable collisionDetected true == true, edge: none', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n4.200, 0, Ego, 95.333, 0.000, 0.000, 0.000, 0.000, 0.000, -1.111, 0.000, -0.767', csv))
        self.assertTrue(re.search('\n4.200, 1, VRU, 89.750, -4.000, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n4.250, 0, Ego, 95.278, 0.000, 0.000, 0.000, 0.000, 0.000, -1.111, 0.000, -0.926', csv))
        self.assertTrue(re.search('\n4.250, 1, VRU, 89.750, -4.000, 0.000, 1.571, 0.000, 0.000, 0.027, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n8.250, 0, Ego, 90.833, 0.000, 0.000, 0.000, 0.000, 0.000, -1.111, 0.000, -1.058', csv))
        self.assertTrue(re.search('\n8.250, 1, VRU, 89.750, 0.057, 0.000, 1.571, 0.000, 0.000, 1.389, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n8.300, 0, Ego, 90.778, 0.000, 0.000, 0.000, 0.000, 0.000, -1.111, 0.000, -1.216', csv))
        self.assertTrue(re.search('\n8.300, 1, VRU, 89.750, 0.127, 0.000, 1.571, 0.000, 0.000, 1.389, 0.000, 0.000', csv))

    def test_AEB_VRU_2023_NCAP_AEB_VRU_CPRA_Cs_2023(self):

        log, duration, cpu_time, _ = run_scenario(os.path.join(NCAP_PREFIX + 'AEB_VRU_2023/NCAP_AEB_VRU_CPRA_Cs_2023.xosc'), COMMON_ARGS)

        # Check some initialization steps
        self.assertTrue(re.search('.*Loading .*NCAP_AEB_VRU_CPRA_Cs_2023', log)  is not None)

        # Check some scenario events
        self.assertTrue(re.search('\n.8.050.* DetectCollision: true, delay: 0.00, 1 collision\\(s\\): Ego and VRU, edge: none', log)  is not None)
        self.assertTrue(re.search('\n.9.100.* storyBoard runningState -> stopTransition -> completeState', log)  is not None)

        # Check vehicle state
        csv = generate_csv()
        self.assertTrue(re.search('\n8.000, 0, Ego, 91.111, 0.000, 0.000, 0.000, 0.000, 0.000, -1.111, 0.000, -0.264', csv))
        self.assertTrue(re.search('\n8.000, 1, VRU, 90.000, 0.000, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv))
        self.assertTrue(re.search('\n8.050, 0, Ego, 91.056, 0.000, 0.000, 0.000, 0.000, 0.000, -1.111, 0.000, -0.423', csv))
        self.assertTrue(re.search('\n8.050, 1, VRU, 90.000, 0.000, 0.000, 4.712, 0.000, 0.000, 0.000, 0.000, 0.000', csv))


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
