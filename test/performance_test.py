import re
import unittest
import argparse
from test_common import *
import matplotlib.pyplot as plt

class TestSuite(unittest.TestCase):
    def test_performance(self):
        applications = ['esmini_releases/esmini_2.32.1/esmini/bin/esmini', 'esmini_releases/esmini_2.34.1/esmini/bin/esmini', 'esmini_releases/esmini_2.35.0/esmini/bin/esmini', 'esmini_releases/esmini_2.39.0/esmini/bin/esmini', 'esmini_releases/esmini_2.45.0/esmini/bin/esmini']
        wall_clock_time = []
        user_time = []
        system_time = []
        cpu_time = []
        for app in applications:
            log, wc_time, u_time, s_time = run_scenario_and_get_time(esmini_arguments='--osc ../resources/xosc/ltap-od_dspace_updated.xosc ' + '--headless --fixed_timestep 0.01 --disable_controllers', application=app)
            wall_clock_time.append(wc_time)
            user_time.append(u_time)
            system_time.append(s_time)
            cpu_time.append(u_time + s_time)
            # Check some initialization steps
            self.assertTrue(re.search('Loading .*ltap-od', log)  is not None)

        print(f"\n----------------------------------------------------------------------------------------------------------------->")
        for app in applications:
            print(f"Application: {app}")
            print(f"Wall clock time: {wall_clock_time[applications.index(app)]:.8f} seconds")
            print(f"User time: {user_time[applications.index(app)]:.8f} seconds")
            print(f"System time: {system_time[applications.index(app)]:.8f} seconds")

        for idx in range(len(applications)):
            print(f"----------------------------------------------------------------------------------------------------------------->")
            print(f"Wall clock time difference between {applications[idx]} and {applications[idx-1]}: {wall_clock_time[idx] - wall_clock_time[idx-1]:.8f} seconds")
            print(f"User time difference between {applications[idx]} and {applications[idx-1]}: {user_time[idx] - user_time[idx-1]:.8f} seconds")
            print(f"System time difference between {applications[idx]} and {applications[idx-1]}: {system_time[idx] - system_time[idx-1]:.8f} seconds")

        fig, axs = plt.subplots(3, 1, figsize=(10, 15))

        # Extracting version numbers from application paths
        app_versions = [re.search(r'esmini_(\d+\.\d+\.\d+)', app).group(1) for app in applications]
        # Subplot for Application vs User Time
        axs[0].plot(app_versions, user_time, marker='o', linestyle='-', color='b')
        axs[0].set_ylabel('User Time (seconds)')
        axs[0].set_title('Application vs User Time')
        axs[0].tick_params(axis='x', rotation=45)
        axs[0].grid(True)

        # Subplot for Application vs System Time
        axs[1].plot(app_versions, system_time, marker='o', linestyle='-', color='g')
        axs[1].set_ylabel('System Time (seconds)')
        axs[1].set_title('Application vs System Time')
        axs[1].tick_params(axis='x', rotation=45)
        axs[1].grid(True)

        # Subplot for Application vs CPU Time
        axs[2].plot(app_versions, cpu_time, marker='o', linestyle='-', color='r')
        axs[2].set_ylabel('CPU Time (User Time + System Time)(seconds)')
        axs[2].set_title('Application vs CPU Time')
        axs[2].tick_params(axis='x', rotation=45)
        axs[2].grid(True)

        plt.tight_layout()
        plt.show()

    def test_performance_advanced(self):
        applications = ['esmini_releases/esmini_2.32.1/esmini/bin/esmini', 'esmini_releases/esmini_2.34.1/esmini/bin/esmini', 'esmini_releases/esmini_2.35.0/esmini/bin/esmini', 'esmini_releases/esmini_2.39.0/esmini/bin/esmini', 'esmini_releases/esmini_2.45.0/esmini/bin/esmini']
        num_iterations = 100
        all_wc_times = {app: [] for app in applications}
        all_user_times = {app: [] for app in applications}
        all_system_times = {app: [] for app in applications}
        all_cpu_times = {app: [] for app in applications}

        for app in applications:
            for _ in range(num_iterations):
                log, wc_time, u_time, s_time = run_scenario_and_get_time(esmini_arguments='--osc ../resources/xosc/ltap-od_dspace_updated.xosc ' + '--headless --fixed_timestep 0.01 --disable_controllers', application=app)
                all_wc_times[app].append(wc_time)
                all_user_times[app].append(u_time)
                all_system_times[app].append(s_time)
                all_cpu_times[app].append(u_time + s_time)
                self.assertTrue(re.search('Loading .*ltap-od', log) is not None)

        fig, axs = plt.subplots(3, 1, figsize=(12, 18))

        app_versions = [re.search(r'esmini_(\d+\.\d+\.\d+)', app).group(1) for app in applications]

        # Plot User Time
        for app, user_times in all_user_times.items():
            axs[0].plot(range(1, num_iterations + 1), user_times, marker='o', label=app)
        axs[0].set_ylabel('User Time (seconds)')
        axs[0].set_title('User Time vs Iteration')
        axs[0].legend()
        axs[0].grid(True)

        # Plot System Time
        for app, system_times in all_system_times.items():
            axs[1].plot(range(1, num_iterations + 1), system_times, marker='o', label=app)
        axs[1].set_ylabel('System Time (seconds)')
        axs[1].set_title('System Time vs Iteration')
        axs[1].legend()
        axs[1].grid(True)

        # Plot CPU Time
        for app, cpu_times in all_cpu_times.items():
            axs[2].plot(range(1, num_iterations + 1), cpu_times, marker='o', label=app)
        axs[2].set_ylabel('CPU Time (User + System)(seconds)')
        axs[2].set_title('CPU Time vs Iteration')
        axs[2].legend()
        axs[2].grid(True)

        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    # execute only if run as a script

    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--timeout", type=int, default=40, help="timeout per testcase")
    parser.add_argument("testcase", nargs="?", help="run only this testcase")
    args = parser.parse_args()

    print("timeout:", args.timeout, file=sys.stderr)
    set_timeout(args.timeout)

    if args.testcase:
        # Add test case name as argument to run only that test case
        unittest.main(argv=['ignored', '-v', 'TestSuite.' + args.testcase])
    else:
        unittest.main(argv=[''], verbosity=2)