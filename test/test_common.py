ESMINI_PATH = '../'

import os
import subprocess
import time
import re
import sys
sys.path.insert(0, ESMINI_PATH + 'scripts')
from dat import *

LOG_FILENAME = 'log.txt'
DAT_FILENAME = 'sim.dat'
CSV_FILENAME = 'sim.csv'
STDOUT_FILENAME = 'stdout.txt'
TIMEOUT = 40


def run_scenario(osc_filename, esmini_arguments, xosc_str = None):

    if os.path.exists(LOG_FILENAME):
        os.remove(LOG_FILENAME)
    if os.path.exists(STDOUT_FILENAME):
        os.remove(STDOUT_FILENAME)

    if osc_filename is not None:
        args = [os.path.join(ESMINI_PATH,'bin','esmini'), '--osc', osc_filename] + esmini_arguments.split()
        #print('running: {}'.format(' '.join(args)))
    elif xosc_str is not None:
        args = [os.path.join(ESMINI_PATH,'bin','esmini')] + esmini_arguments.split() + ['--osc_str', xosc_str]

    return_code = None
    with open(STDOUT_FILENAME, "w") as f:
        process = subprocess.Popen(args, cwd=os.path.dirname(os.path.realpath(__file__)),
                            stdout=f)

        elapsed = 0

        while elapsed < TIMEOUT and return_code is None:

            return_code = process.poll()

            # watch dog
            if return_code is None:
                time.sleep(1)
                elapsed += 1

        if return_code is None:
            print('timeout ({}s). Terminating scenario ({}).'.format(TIMEOUT, os.path.basename(osc_filename)))
            process.kill()
            assert False, 'Timeout'

    assert return_code == 0

    with open(LOG_FILENAME, 'r') as logfile:
        return logfile.read()

    assert False, 'No log file'


def generate_csv(mode_ = "original", time_step_ = 0.05):

    # Below is one/the old way of converting dat to csv. Keeping the lines for reference.
    # args = [os.path.join(ESMINI_PATH,'bin','dat2csv'), DAT_FILENAME]
    # process = subprocess.run(args, cwd=os.path.dirname(os.path.realpath(__file__)))

    # Below is the Python way of converting dat to csv
    dat = DATFile(DAT_FILENAME)
    dat.save_csv(mode = mode_, step_time = time_step_)

    with open(CSV_FILENAME, "r") as f:
        return f.read()

    assert False, 'No csv file'
