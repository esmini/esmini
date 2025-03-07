import sys
import unittest
import argparse
import os.path
from test_common import *

ESMINI_PATH = '../'
COMMON_ESMINI_ARGS = '--headless --fixed_timestep 0.01 --record sim.dat '
TOLERANCE = 0.1

class TestSuite(unittest.TestCase):

    def run_repeat(self, app, args, n_runs, ref_duration, ref_cpu_time):
        total_duration = 0
        total_cpu_time = 0
        for i in range(n_runs):
            log, duration, cpu_time = run_scenario(app, args, measure_cpu_time=True)
            total_duration += duration
            total_cpu_time += (cpu_time.user + cpu_time.system)

        print('duration {:.3f} (ref {:.3f}) cpu_time {:.3f} (ref {:.3f})'.format(
            total_duration / n_runs, ref_duration,
            total_cpu_time / n_runs, ref_cpu_time))

        self.assertLess(total_duration / n_runs, ref_duration * (1 + TOLERANCE))
        self.assertLess(total_cpu_time / n_runs, ref_cpu_time * (1 + TOLERANCE))

    def test_perf_ltap_od(self):
        self.run_repeat(os.path.join(ESMINI_PATH, 'resources/xosc/ltap-od.xosc'), COMMON_ESMINI_ARGS + '--disable_controllers', 10, 0.25, 0.04)

    def test_perf_swarm(self):
        self.run_repeat(os.path.join(ESMINI_PATH, 'resources/xosc/swarm.xosc'), COMMON_ESMINI_ARGS + ' --seed 0' + ' --fixed_timestep 0.1', 5, 1.4, 0.7)


if __name__ == "__main__":
    # execute only if run as a script

    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--timeout", type=int, default=40, help="timeout per testcase")
    parser.add_argument("testcase", nargs="?", help="run only this testcase")
    args = parser.parse_args()

    print("timeout:", args.timeout, file=sys.stderr)
    print("Tolerance: {:.2f} ({:.0f}%)".format(TOLERANCE, 100 * TOLERANCE))
    set_timeout(args.timeout)

    if args.testcase:
        # Add test case name as argument to run only that test
        # example: smoke_test.py test_follow_ghost
        unittest.main(argv=['ignored', '-v', 'TestSuite.' + args.testcase])
    else:
        unittest.main(argv=[''], verbosity=2)
