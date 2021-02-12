import os
import subprocess
import time
import re
import sys
import unittest

LOG_FILENAME = 'log.txt'
DAT_FILENAME = 'sim.dat'
CSV_FILENAME = 'sim.csv'
STDOUT_FILENAME = 'stdout.txt'
ESMINI_PATH = '../'
TIMEOUT = 10
COMMON_ARGS = '--headless --fixed_timestep 0.01 --record sim.dat '


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


class TestSuite(unittest.TestCase):
    
    def test_cut_in(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/cut-in.xosc'), COMMON_ARGS)
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading ../resources/xosc/cut-in.xosc', log))
        
        # Check some scenario events
        self.assertTrue(re.search('0.010 .* CutInActStart == true, 0.0100 > 0.00 edge: NONE', log))
        self.assertTrue(re.search('\n[678].* BrakeCondition == true, HWT: 0.70 > 0.70, edge Rising', log))
        self.assertTrue(re.search('\n21.* ActStopCondition timer expired at 5.00 seconds', log))
        

    def test_ltap_od(self):
        log = run_scenario(os.path.join(ESMINI_PATH, 'resources/xosc/ltap-od.xosc'), COMMON_ARGS \
            + '--disable_controllers')
        
        # Check some initialization steps
        self.assertTrue(re.search('.*Loading ../resources/xosc/ltap-od.xosc', log))
        self.assertTrue(re.search('.*Route::AddWaypoint Added connecting waypoint 1: 9, -1, 0.00', log))

        # Check some scenario events
        self.assertTrue(re.search('\n5.5.*Synchronize dist \(0.95\) < tolerance \(1.00\)', log))
        self.assertTrue(re.search('\n9.5.* QuitCondition timer expired at 4.0. seconds', log))

        # Check vehicle key positions
        csv = generate_csv()
        self.assertTrue(re.search('\n6.500, 0, Ego, 28.5., -7.8., 0.00, 1.[78].', csv))
        self.assertTrue(re.search('\n6.500, 1, NPC, 24.5., 0.2., 0.00, 5.[34].', csv))


if __name__ == "__main__":
    # execute only if run as a script

    unittest.main(verbosity=2)
