import sys
import unittest
import argparse
import os.path
from test_common import *

ESMINI_PATH = '../'
COMMON_ESMINI_ARGS = '--headless --fixed_timestep 0.01 --record sim.dat '
TOLERANCE = 0.4

class TestSuite(unittest.TestCase):

    def run_repeat(self, app, args, ref_cpu_time, n_runs):

        if n_runs < 1:
            self.assertFalse, "No runs specified"
            return

        total_duration = 0
        total_cpu_user = 0
        total_cpu_system = 0
        total_cpu_total = 0
        result_cpu_user = []
        result_cpu_system = []
        result_cpu_total = []

        for i in range(n_runs):
            log, duration, cpu_time = run_scenario(app, args, measure_cpu_time=True)
            total_duration += duration
            total_cpu_user += cpu_time.user
            total_cpu_system += cpu_time.system
            total_cpu_total += (cpu_time.user + cpu_time.system)
            result_cpu_user.append(cpu_time.user)
            result_cpu_system.append(cpu_time.system)
            result_cpu_total.append(cpu_time.user + cpu_time.system)

        average_duration = total_duration / n_runs
        average_cpu_user = total_cpu_user / n_runs
        average_cpu_system = total_cpu_system / n_runs
        average_cpu_total = total_cpu_total / n_runs

        result_cpu_user.sort()
        result_cpu_system.sort()
        result_cpu_total.sort()

        median_cpu_user = result_cpu_user[int(n_runs / 2)] if n_runs % 2 else result_cpu_user[int(n_runs / 2 - 1)]
        median_cpu_system = result_cpu_system[int(n_runs / 2)] if n_runs % 2 else result_cpu_system[int(n_runs / 2 - 1)]
        median_cpu_total = result_cpu_total[int(n_runs / 2)] if n_runs % 2 else result_cpu_total[int(n_runs / 2 - 1)]

        self.assertLess(median_cpu_total, ref_cpu_time * (1 + TOLERANCE))

        print('{:.3f} ({:.3f}) / {:.3f}, {:.3f} / {:.3f}, {:.3f} / {:.3f}, {:.3f}, {}: '.format(
            median_cpu_total, ref_cpu_time, average_cpu_total, median_cpu_user, average_cpu_user, median_cpu_system, average_cpu_system, average_duration, n_runs), flush=True, end='')

    def test_perf_cut_in(self):
        self.run_repeat(os.path.join(ESMINI_PATH, 'resources/xosc/cut-in.xosc'), COMMON_ESMINI_ARGS + '--disable_controllers --fixed_timestep 0.001', 0.19, 40)

    def test_perf_ltap_od(self):
        self.run_repeat(os.path.join(ESMINI_PATH, 'resources/xosc/ltap-od.xosc'), COMMON_ESMINI_ARGS + '--disable_controllers --fixed_timestep 0.001', 0.15, 40)

    def test_perf_swarm(self):
        self.run_repeat(os.path.join(ESMINI_PATH, 'resources/xosc/swarm.xosc'), COMMON_ESMINI_ARGS + ' --seed 0' + ' --fixed_timestep 0.1', 1.35, 10)


if __name__ == "__main__":
    # execute only if run as a script

    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--timeout", type=int, default=40, help="timeout per testcase")
    parser.add_argument("testcase", nargs="?", help="run only this testcase")
    args = parser.parse_args()

    print("timeout:", args.timeout, flush=True)
    print("Tolerance: {:.2f} ({:.0f}%)".format(TOLERANCE, 100 * TOLERANCE), flush=True)
    print("Name ... cpu_total median (ref) / average, cpu_user median / average, cpu_system median / average, duration average, n_runs: Result", flush=True)
    set_timeout(args.timeout)

    if args.testcase:
        # Add test case name as argument to run only that test
        # example: smoke_test.py test_follow_ghost
        unittest.main(argv=['ignored', '-v', 'TestSuite.' + args.testcase])
    else:
        unittest.main(argv=[''], verbosity=2)
