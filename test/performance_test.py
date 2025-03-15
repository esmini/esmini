import unittest
import sys
import argparse
import os
import glob
from test_common import *

ESMINI_PATH = '../'
COMMON_ESMINI_ARGS = '--headless --fixed_timestep 0.01 --record sim.dat '
TOLERANCE = 0.4
directories = []
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

    def run_repeat(self, scenario, args, ref_cpu_time, n_runs, plot=False):

        if n_runs < 1:
            self.assertFalse, "No runs specified"
            return

        time_vals_run = []
        remove_dirs = []

        for i, dir in enumerate(directories):

            exec = None if dir is None else os.path.join(dir, "bin/esmini")
            total_duration = 0
            total_cpu_user = 0
            total_cpu_system = 0
            total_cpu_total = 0
            result_cpu_user = []
            result_cpu_system = []
            result_cpu_total = []

            failure = False
            for j in range(n_runs):
                try:
                    log, duration, cpu_time = run_scenario(scenario, args, application=exec, measure_cpu_time=True)
                except:
                    print('\nFailure - check', exec, file=sys.stderr, flush=True)
                    remove_dirs.append(dir)
                    failure = True
                    break
                total_duration += duration
                total_cpu_user += cpu_time.user
                total_cpu_system += cpu_time.system
                total_cpu_total += (cpu_time.user + cpu_time.system)
                result_cpu_user.append(cpu_time.user)
                result_cpu_system.append(cpu_time.system)
                result_cpu_total.append(cpu_time.user + cpu_time.system)

            if failure:
                continue
            average_duration = total_duration / n_runs
            average_cpu_user = total_cpu_user / n_runs
            average_cpu_system = total_cpu_system / n_runs
            average_cpu_total = total_cpu_total / n_runs

            result_cpu_user.sort()
            result_cpu_system.sort()
            result_cpu_total.sort()

            median_cpu_user = self.median(result_cpu_user)
            median_cpu_system = self.median(result_cpu_system)
            median_cpu_total = self.median(result_cpu_total)

            if ref_cpu_time > 0:
                self.assertLess(median_cpu_total, ref_cpu_time * (1 + TOLERANCE))

            print('\n{} {} {:.3f} ({:.3f}) / {:.3f}, {:.3f} / {:.3f}, {:.3f} / {:.3f}, {:.3f}, {} '.format(
                os.path.realpath(exec).replace(os.path.sep, '/'),
                os.path.basename(scenario),
                median_cpu_total, ref_cpu_time, average_cpu_total, median_cpu_user, average_cpu_user, median_cpu_system, average_cpu_system, average_duration, n_runs),
                flush=True, end='')

            time_vals_run.append(median_cpu_total)

        for d in remove_dirs:
            directories.remove(d)
        print('', flush=True)
        time_vals.append(time_vals_run)
        return 0

class TestSuitePresetScenarios(TestSuiteBase):

    def test_perf_cut_in(self):
        scenario = 'resources/xosc/cut-in.xosc'
        scenarios.append(scenario)
        self.run_repeat(os.path.join(ESMINI_PATH, scenario), COMMON_ESMINI_ARGS + '--disable_controllers --fixed_timestep 0.001', 0.19, 40 if args.runs < 0 else args.runs)

    def test_perf_ltap_od(self):
        scenario = 'resources/xosc/ltap-od.xosc'
        scenarios.append(scenario)
        self.run_repeat(os.path.join(ESMINI_PATH, scenario), COMMON_ESMINI_ARGS + '--disable_controllers --fixed_timestep 0.001', 0.15, 40 if args.runs < 0 else args.runs)

    def test_perf_swarm(self):
        scenario = 'resources/xosc/swarm.xosc'
        scenarios.append(scenario)
        self.run_repeat(os.path.join(ESMINI_PATH, scenario), COMMON_ESMINI_ARGS + ' --seed 0' + ' --fixed_timestep 0.1', 1.35, 10 if args.runs < 0 else args.runs)

class TestSuiteSpecifiedScenarios(TestSuiteBase):

    def test_scenario_list(self):
        for s in scenarios:
            self.run_repeat(os.path.join(ESMINI_PATH, s), COMMON_ESMINI_ARGS + ' --seed 0' + ' --fixed_timestep 0.05 --disable_controllers ', -1, args.runs)

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

def Plot():
    dirs = [os.path.basename(os.path.normpath(d).replace(os.path.sep, '/')) for d in directories]
    labels = [os.path.basename(s) for s in scenarios]

    for i, time_val in enumerate(time_vals):
        plt.plot(dirs, time_val, label=labels[i], marker='o')

    plt.ylabel('CPU time (seconds)')
    plt.title('esmini performance')
    plt.tick_params(axis='x', rotation=45)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    # Save the plot as an image
    plt.savefig("esmini_performance.png")  # Saves as PNG by default

    if args.show_plot == True:
        plt.show()

if __name__ == "__main__":
    # execute only if run as a script

    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--timeout", type=int, default=40, help="timeout per testcase")
    parser.add_argument("-d", "--directories", nargs="+", type=str, help="list of esmini root directories")
    parser.add_argument("-s", "--scenarios", nargs="+", type=str, help="list of scenarios (if missing default suite is run)")
    parser.add_argument("-r", "--runs", type=int, help="number of runs (overrides default settings per test)", default=20)
    parser.add_argument("-p", "--show_plot", nargs="?", const=True, default=None, help="enforce show plot (otherwise only with -s)")
    parser.add_argument("--disable_plot", action="store_true", default=False, help="disable plot (not even store file)")
    parser.add_argument("testcase", nargs="?", help="run only this testcase")
    args = parser.parse_args()

    if args.directories is None:
        directories.append(os.path.realpath('../'))  # default path

    if args.directories is not None:
        directories += [os.path.realpath(d) for d in expand_wildcards(args.directories)]

    if args.scenarios is not None:
        scenarios += [os.path.realpath(s) for s in expand_wildcards(args.scenarios)]

    print("timeout:", args.timeout, flush=True)
    print("tolerance: {:.2f} ({:.0f}%)".format(TOLERANCE, 100 * TOLERANCE), flush=True)
    if len(scenarios) > 0:
        print('runs:', args.runs, flush=True)

    print("executable scenario ... cpu_total median (ref) / average, cpu_user median / average, cpu_system median / average, duration average, n_runs", flush=True)

    set_timeout(args.timeout)

    if not args.disable_plot:
        import matplotlib.pyplot as plt
        unittest.addModuleCleanup(Plot)
        if args.show_plot is None:
            args.show_plot = True if args.scenarios else False

    if args.scenarios:
        # run specified scenarios
        unittest.main(argv=['', 'TestSuiteSpecifiedScenarios'], verbosity=2)
    elif args.testcase:
        # run specified test cases from the preset selection
        unittest.main(argv=['', 'TestSuitePresetScenarios.' + args.testcase], verbosity=2)
    else:
        # run complete preset selection
        unittest.main(argv=['', 'TestSuitePresetScenarios'], verbosity=2)


