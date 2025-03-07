"""
This script checks esmini performance by measuring CPU execution time.

Two modes:
   1. Preset selection of scenarios with fixed timestep settings.
      This mode is useful for benchmarking build against a previous version. Used in CI.

      Example 1a - preset performance test:
         python ./performance_test.py

      Example 1b - control confidence by specifying nr runs:
         python ./performance_test.py -r 50

      Example 1c - run benchmark for multiple executables:
         python ./performance_test.py -e /tmp/esmini_branch1 /tmp/esmini_branch2

   2. Custom selection of scenarios and global timestep setting.
      This mode is useful for finding performance regression.

      Example 2a - check multiple esmini versions (in folders 2_32, 2_33..., 2_40, 2_41...) with one scenario
         python ./performance_test.py -s ../resources/xosc/ltap-od.xosc -e C:/tmp/esmini-2_3[23456789]* C:/tmp/esmini-2_4*

      Example 2b - check multiple esmini versions (in folders 2_42 and 2_43) with two scenarios
         python ./performance_test.py -s ../resources/xosc/ltap-od.xosc ../resources/xosc/cut-in.xosc -e C:/tmp/esmini-2_4[23]

      Example 2c - Mix reference to executables and directories
         python ./performance_test.py -s ../resources/xosc/cut-in.xosc -e /tmp/performance_test/esmini-c59163c1/bin/esmini ../

      Example 2d - Add custom esmini arguments (make sure to enclose multiple arguments in quotes "")
         python ./performance_test.py -s ../resources/xosc/ltap-od.xosc --esmini_args "--disable_controllers --log_level debug"

Plot is created and saved to file by default. In mode 2 plot is also shown on screen.

Dependencies:
    unittest
    matplotlib (optional, see -h/--help)
"""

import unittest
import sys
import argparse
import os
import glob
from test_common import *

ESMINI_PATH = '../'
COMMON_ESMINI_ARGS = '--headless --record sim.dat --seed 0'
TOLERANCE = 0.4
executables = []
scenarios = []
time_vals = []

class TestSuiteBase(unittest.TestCase):

    def median(self, list):

        size = len(list)

        if size == 0:
            return 0

        if size % 2:
            # odd number of elements
            return list[int(size/2)]
        else:
            # even number of elements
            return (list[int(size/2 - 1)] + list[int(size/2)]) / 2

    def run_repeat(self, scenario, esmini_args, timestep, ref_cpu_time, n_runs, plot=False):

        if n_runs < 1:
            self.assertFalse, "No runs specified"
            return

        time_vals_run = []
        if timestep is None:
            timestep = 0.1

        esmini_args += ' --fixed_timestep ' + str(timestep) + " " + " ".join(args.esmini_args)

        for i, exec in enumerate(executables):

            result_duration = []
            result_cpu_user = []
            result_cpu_system = []
            result_cpu_total = []

            failure = False
            for j in range(n_runs):
                try:
                    log, duration, cpu_time = run_scenario(scenario, esmini_args, application=exec, measure_cpu_time=True)
                except Exception as e:
                    print(e, flush=True)
                    print('\nFailure - check log.txt and', exec, file=sys.stderr, flush=True)
                    self.fail("Failed to execute test")
                    break
                result_duration.append(duration)
                result_cpu_user.append(cpu_time.user)
                result_cpu_system.append(cpu_time.system)
                result_cpu_total.append(cpu_time.user + cpu_time.system)

            if failure:
                continue

            if n_runs > 1:
                # remove first run to reduce irrelevant variance
                del result_duration[0]
                del result_cpu_user[0]
                del result_cpu_system[0]
                del result_cpu_total[0]

            average_duration = sum(result_duration) / len(result_duration)
            average_cpu_user = sum(result_cpu_user) / len(result_cpu_user)
            average_cpu_system = sum(result_cpu_system) / len(result_cpu_system)
            average_cpu_total = sum(result_cpu_total) / len(result_cpu_total)

            result_cpu_user.sort()
            result_cpu_system.sort()
            result_cpu_total.sort()

            median_cpu_user = self.median(result_cpu_user)
            median_cpu_system = self.median(result_cpu_system)
            median_cpu_total = self.median(result_cpu_total)

            if ref_cpu_time > 0:
                self.assertLess(median_cpu_total, ref_cpu_time * (1 + TOLERANCE))

            print('\n{}, {} {:.3f} ({:.3f}) / {:.3f}, {:.3f} / {:.3f}, {:.3f} / {:.3f}, {:.3f}, {}, {} '.format(
                os.path.realpath(exec).replace(os.path.sep, '/'),
                os.path.basename(scenario),
                median_cpu_total, ref_cpu_time, average_cpu_total, median_cpu_user, average_cpu_user, median_cpu_system, average_cpu_system, average_duration, timestep, n_runs),
                flush=True, end='')

            time_vals_run.append(median_cpu_total)

        print('', flush=True)
        time_vals.append(time_vals_run)

class TestSuitePresetScenarios(TestSuiteBase):

    def test_perf_cut_in(self):
        scenario = 'resources/xosc/cut-in.xosc'
        scenarios.append(scenario)
        self.run_repeat(os.path.join(ESMINI_PATH, scenario), COMMON_ESMINI_ARGS, 0.001, 0.18, 40 if args.runs < 0 else args.runs)

    def test_perf_ltap_od(self):
        scenario = 'resources/xosc/ltap-od.xosc'
        scenarios.append(scenario)
        self.run_repeat(os.path.join(ESMINI_PATH, scenario), COMMON_ESMINI_ARGS + " --disable_controllers", 0.001, 0.10, 40 if args.runs < 0 else args.runs)

    def test_perf_swarm(self):
        scenario = 'resources/xosc/swarm.xosc'
        scenarios.append(scenario)
        self.run_repeat(os.path.join(ESMINI_PATH, scenario), COMMON_ESMINI_ARGS, 0.1, 1.32, 10 if args.runs < 0 else args.runs)

class TestSuiteSpecifiedScenarios(TestSuiteBase):

    def test_scenario_list(self):
        for s in scenarios:
            self.run_repeat(os.path.join(ESMINI_PATH, s), COMMON_ESMINI_ARGS, args.timestep, -1, args.runs)

def expand_wildcards(args):
    expanded_files = []
    for arg in args:
        files = glob.glob(arg)
        if files:
            expanded_files.extend(files)  # Add found files to the list
        else:
            # If no files match the argument, treat it as a literal argument
            expanded_files.append(arg)
    return expanded_files

def remove_identical_shared_prefixes_suffixes(strings):
    min_prefix_len = min(len(s) for s in strings)
    min_suffix_len = min(len(s) for s in strings)

    # Find the longest shared prefix length
    prefix_len = 0
    for i in range(1, min_prefix_len + 1):
        prefix = strings[0][:i]
        if all(s.startswith(prefix) for s in strings):
            prefix_len = i
        else:
            break

    # Find the longest shared suffix length
    suffix_len = 0
    for i in range(1, min_suffix_len + 1):
        suffix = strings[0][-i:]
        if all(s.endswith(suffix) for s in strings):
            suffix_len = i
        else:
            break

    # Remove the shared prefixes and suffixes
    result = [s[prefix_len:len(s) - suffix_len] for s in strings]
    return result

def Plot():
    if len(executables) == len(time_vals):
        execs = remove_identical_shared_prefixes_suffixes([os.path.normpath(e).replace(os.path.sep, '/') for e in executables])
        labels = [os.path.basename(s) for s in scenarios]

        for i, time_val in enumerate(time_vals):
            plt.plot(execs, time_val, label=labels[i], marker='o')

        plt.ylabel('CPU time (seconds)')
        plt.title('esmini performance')
        plt.tick_params(axis='x', rotation=45)
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

        # Save the plot as an image
        plt.savefig("performance.png")  # Saves as PNG by default

        if args.hide_plot == False:
            plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--executables", nargs="+", type=str, help="list of esmini executables and/or esmini root directories, regex supported")
    parser.add_argument("-s", "--scenarios", nargs="+", type=str, help="list of scenarios (if missing, run default selection), regex supported")
    parser.add_argument("-r", "--runs", type=int, help="number of runs (overrides per test setting) (default: %(default)s)", default=20)
    parser.add_argument("--timestep", default=0.05, help="timestep (only with -s, overrides per test setting) (default: %(default)s)")
    parser.add_argument("--hide_plot", action="store_true", help="do not show plot (just save to file)")
    parser.add_argument("--disable_plot", action="store_true", help="disable plot (not even store file)")
    parser.add_argument("-t", "--timeout", type=int, default=40, help="timeout per testcase (default: %(default)s)")
    parser.add_argument("-a", "--esmini_args", nargs=1, type=str, default="", help="additional esmini arguments (multiple args within \"\")")
    parser.add_argument("testcase", nargs="?", help="run only this testcase")
    args = parser.parse_args()

    if args.executables is None:
        executables.append(os.path.realpath('../bin/esmini'))  # default path

    if args.executables is not None:
        for path in expand_wildcards(args.executables):
            if os.path.isdir(path):
                # assume path is esmini root, and executable at ./bin/esmini
                executables.append(os.path.realpath(path + os.sep + './bin/esmini'))
            else:
                executables.append(os.path.realpath(path))

    if args.scenarios is not None:
        scenarios += [os.path.realpath(s) for s in expand_wildcards(args.scenarios)]

    print("timeout:", args.timeout, flush=True)
    print("tolerance: {:.2f} ({:.0f}%)".format(TOLERANCE, 100 * TOLERANCE), flush=True)
    if len(scenarios) > 0:
        print('runs:', args.runs, flush=True)

    print("executable, scenario ... cpu_total median (ref) / average, cpu_user median / average, cpu_system median / average, duration average, timestep, n_runs", flush=True)

    set_timeout(args.timeout)

    if not args.disable_plot:
        import matplotlib.pyplot as plt
        unittest.addModuleCleanup(Plot)

    if args.scenarios:
        # run specified scenarios
        unittest.main(argv=['', 'TestSuiteSpecifiedScenarios'], verbosity=2)
    elif args.testcase:
        # run specified test cases from the preset selection
        unittest.main(argv=['', 'TestSuitePresetScenarios.' + args.testcase], verbosity=2)
    else:
        # run complete preset selection
        unittest.main(argv=['', 'TestSuitePresetScenarios'], verbosity=2)

