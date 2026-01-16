import os
import subprocess
import time
import sys
import psutil

ESMINI_PATH = os.path.realpath(os.pardir)

sys.path.insert(0, os.path.join(ESMINI_PATH, 'scripts'))

LOG_FILENAME = 'log.txt'
DAT_FILENAME = 'sim.dat'
OSI_FILENAME = 'ground_truth.osi'
STDOUT_FILENAME = 'stdout.txt'
TIMEOUT = 40
POLL_INTERVAL = 0.1  # seconds

VALGRIND_LEVEL_SUMMARY = ["valgrind", "--leak-check=summary", "--show-leak-kinds=definite,possible"]
VALGRIND_LEVEL_FULL = ["valgrind", "--leak-check=full", "--show-leak-kinds=all", "--track-origins=yes", "--verbose"]
# Add path to esmini shared library
# needed only on Mac and Linux, Windows looks in folder of the executable
env = os.environ.copy()
if sys.platform == "linux" or sys.platform == "linux2":
    env['LD_LIBRARY_PATH'] = ':'.join([env.get('LD_LIBRARY_PATH', ''), os.path.join(ESMINI_PATH, 'bin')])
elif sys.platform == "darwin":
    env['DYLD_LIBRARY_PATH'] = ':'.join([env.get('DYLD_LIBRARY_PATH', ''), os.path.join(ESMINI_PATH, 'bin')])


def set_timeout(timeout):
    global TIMEOUT
    TIMEOUT = timeout

def run_scenario(osc_filename = None, esmini_arguments = None, xosc_str = None, application = None, ignoreReturnCode = False, measure_cpu_time = False, check_memory_leaks_level: str = "none", print_duration = True):

    if os.path.exists(LOG_FILENAME):
        os.remove(LOG_FILENAME)
    if os.path.exists(STDOUT_FILENAME):
        os.remove(STDOUT_FILENAME)

    if application is None:
        app = os.path.join(ESMINI_PATH, 'bin', 'esmini')
    elif os.path.isabs(application) == False:
        app = os.path.join(ESMINI_PATH, application)
    else:
        app = application

    # add postfix .exe for windows search path, allowing for parallel build existence
    app += '.exe' if sys.platform == "win32" else ''

    if osc_filename is not None:
        args = [app, '--osc', osc_filename] + esmini_arguments.split()
    else:
        args = [app] + esmini_arguments.split()
        if xosc_str is not None:
            args +=  ['--osc_str', xosc_str]

    if check_memory_leaks_level == "summary":
        args = VALGRIND_LEVEL_SUMMARY + args
    elif check_memory_leaks_level == "full":
        args = VALGRIND_LEVEL_FULL + args

    return_code = None
    cpu_times = None
    start_time = None

    with open(STDOUT_FILENAME, "w") as f:
        process = subprocess.Popen(args, cwd=os.path.dirname(os.path.realpath(__file__)),
                            stdout=f, stderr=subprocess.STDOUT, env=env)
        ps_process = psutil.Process(process.pid)
        start_time = time.time()
        while time.time() - start_time < TIMEOUT and return_code is None:
            return_code = process.poll()
            if measure_cpu_time:
                try:
                    # we want the total execution time
                    # however, after process terminated we can't get time info
                    # hence we need to continuously fetch it
                    cpu_times = ps_process.cpu_times()
                except:
                    pass
            else:
                time.sleep(POLL_INTERVAL)

        if return_code is None:
            print('timeout ({}s). Terminating scenario ({}).'.format(time.time() - start_time, os.path.basename(osc_filename)))
            process.kill()
            process.wait()
            assert False, 'Timeout'

    duration = time.time() - start_time

    if not ignoreReturnCode:
        assert return_code == 0

    if print_duration:
        print('({:.2f} s) '.format(duration), end='', file=sys.stderr, flush=True)

    errlog = None
    with open(STDOUT_FILENAME, 'r') as errfile:
        errlog = errfile.read()

    with open(LOG_FILENAME, 'r') as logfile:
        log = logfile.read()
        return log, duration, cpu_times, errlog

    assert False, 'No log file'

def run_replayer(replayer_arguments = None):

    if os.path.exists(LOG_FILENAME):
        os.remove(LOG_FILENAME)
    if os.path.exists(STDOUT_FILENAME):
        os.remove(STDOUT_FILENAME)

    app = os.path.join(ESMINI_PATH,'bin','replayer')
    return_code = None
    args = [app] + replayer_arguments.split()
    with open(STDOUT_FILENAME, "w") as f:
        process = subprocess.Popen(args, cwd=os.path.dirname(os.path.realpath(__file__)), stdout=f, env=env)

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

    with open(STDOUT_FILENAME, 'r') as logfile:
        log = logfile.read()
        assert return_code == 0, log
        return log

    assert False, 'No log file'

def use_package(pack_name):
    result = subprocess.run(
        ["cmake", "-B", "../build", "-N", "-L"],
        capture_output=True,
        text=True,
        check=True,
        shell=False
    )
    return result.stdout.find("USE_" + pack_name + ":BOOL=ON") != -1

def generate_csv(filename=DAT_FILENAME):

    with open(STDOUT_FILENAME, "w") as f:
        args = [os.path.join(ESMINI_PATH,'bin','dat2csv'), "--file", filename]
        process = subprocess.Popen(args, cwd=os.path.dirname(os.path.realpath(__file__)), stdout=f)
        assert process.wait() == 0

    with open(os.path.splitext(filename)[0] + '.csv', "r") as f:
        return f.read()

    assert False, 'No csv file'

def generate_csv_from_osi(filename=OSI_FILENAME):

    if use_package("OSI"):
        import osi2csv

        osi = osi2csv.OSIFile(filename)
        osi.save_csv()
        osi.close()

        with open(os.path.splitext(filename)[0] + '.csv', "r") as f:
            return f.read()

        assert False, 'No csv file'
