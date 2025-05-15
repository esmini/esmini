import re
import sys
from test_common import *
import unittest
import argparse
import os.path

ESMINI_PATH = '../'
COMMON_ESMINI_ARGS = '--headless --fixed_timestep 0.1 --record sim.dat --osi_file --osi_crop_dynamic 0,300 '

class TestSuite(unittest.TestCase):
    pass

def make_memory_test(path: str, sce: str, level: int):
    def test_func(self):
        _, _, _, errlog = run_scenario(os.path.join(path, sce), COMMON_ESMINI_ARGS + '--log_level debug', None, None, False, False, level)

        # Check for memory leaks
        self.assertIsNotNone(re.search('.definitely lost. 0 bytes in 0 blocks', errlog))
        self.assertIsNotNone(re.search('.indirectly lost. 0 bytes in 0 blocks', errlog))
        self.assertIsNotNone(re.search('.possibly lost. 0 bytes in 0 blocks', errlog))

    return test_func

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="""
        Test memory leaks in scenarios using esmini. Default is leak_level=summary,
        """
    )
    parser.add_argument("-t", "--timeout", type=int, default=600, help="timeout per testcase")
    parser.add_argument("-p", "--path", nargs="+", default=["resources/xosc/"], help="relative path to the root directory")
    parser.add_argument("-b", "--blacklist", nargs="+", default=None, help="exclude these scenarios from the test")
    parser.add_argument("-s", "--scenario", nargs="?", help="run only this scenario with full analysis (will ignore blacklist)")
    
    args = parser.parse_args()

    if args.scenario: # If we want to run specific scenario, ignore the blacklist
        args.blacklist = None

    print("timeout:", args.timeout, file=sys.stderr)
    set_timeout(args.timeout)

    # Loop over all paths given by the user
    # Then loop over all scenarios in each path
    # If scenario is in blacklist, skip it
    for scenario_path in args.path:
        scenario_path = os.path.join(ESMINI_PATH, scenario_path)
        if not os.path.exists(scenario_path):
            print(f"Path {scenario_path} does not exist. Skipping.")
            continue

        for scenario in os.listdir(scenario_path):
            if not scenario.endswith(".xosc") or args.blacklist is not None and scenario in args.blacklist:
                continue

            test_name = f"test_{os.path.splitext(scenario)[0]}"
            check_level = "full" if args.scenario else "summary"
            setattr(TestSuite, test_name, make_memory_test(scenario_path, scenario, check_level))

    if args.scenario:
        # Add test case name as argument to run only that test
        # example: smoke_test.py test_follow_ghost
        test_name = f"test_{os.path.splitext(args.scenario)[0]}"
        unittest.main(argv=['ignored', '-v', f'TestSuite.{test_name}'])
    else:
        unittest.main(argv=[''], verbosity=2)
