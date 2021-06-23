import os
import subprocess
import time
import re
import sys

LOG_FILENAME = 'log.txt'
DAT_FILENAME = 'sim.dat'
CSV_FILENAME = 'sim.csv'
STDOUT_FILENAME = 'stdout.txt'
TIMEOUT = 40
ESMINI_PATH = '../'


def run_scenario(osc_filename, esmini_arguments):
    
    if os.path.exists(LOG_FILENAME):
        os.remove(LOG_FILENAME)
    if os.path.exists(STDOUT_FILENAME):
        os.remove(STDOUT_FILENAME)

    args = [os.path.join(ESMINI_PATH,'bin','esmini'), '--osc', osc_filename] + esmini_arguments.split()
    #print('running: {}'.format(' '.join(args)))
    
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


def generate_csv():
    args = [os.path.join(ESMINI_PATH,'bin','dat2csv'), DAT_FILENAME]
    process = subprocess.run(args, cwd=os.path.dirname(os.path.realpath(__file__)))
    with open(CSV_FILENAME, "r") as f:
        return f.read()

    assert False, 'No csv file'


