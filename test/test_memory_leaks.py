import re
import sys
from sys import platform
from test_common import *
import unittest
import argparse
import os.path

ESMINI_PATH = '../'
COMMON_ESMINI_ARGS = '--headless --fixed_timestep 0.1 --record sim.dat '
COMMON_REPLAYER_ARGS = '--file sim.dat --headless --time_scale 10 --res_path ../resources --quit_at_end '


class TestSuite(unittest.TestCase):

    def test_all_example_scenarios(self):
        failures = []
        xosc_files = [f for f in os.listdir(os.path.join(ESMINI_PATH, 'resources/xosc')) if f.endswith('.xosc')]
        for scenario in xosc_files[0:2]:
            skip_file = False
            disable_controllers = False

            with open(os.path.join(ESMINI_PATH, "resources/xosc", scenario), "r", encoding="utf-8") as xosc:
                for line in xosc:
                    if "ParameterDistribution" in line:
                        print(f"  ⚠️ Skipping '{scenario}' (contains 'ParameterDistribution')")
                        skip_file = True
                        break
                    if "interactiveDriver" in line or "InteractiveController" in line:
                        disable_controllers = True

            if skip_file:
                continue

            EXTRA_ARGS = "--disable_controllers " if disable_controllers else ""
            log, duration, cpu_time = run_scenario(os.path.join(ESMINI_PATH, f'resources/xosc/{scenario}'), COMMON_ESMINI_ARGS + EXTRA_ARGS + '--log_level debug', None, None, False, False, True)

            # Check some initialization steps
            try:
                self.assertTrue(re.search(f'Loading .*{scenario}', log) is not None)
            except AssertionError as e:
                failures.append(f"Scenario {scenario}: {str(e)}")

            # Final report
        if failures:
            failure_summary = "\n".join(failures)
            self.fail(f"\nSome scenario checks failed:\n{failure_summary}")
        # Check some scenario events

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
