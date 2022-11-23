
import subprocess
import sys
import threading
import time

launched = 0
done = 0
n_runs = 0
ps = []

def print_status():
    print('Launched: {}/{} Done: {}'.format(launched, n_runs, done), end='\r', flush=True)

def launch_func():
    global n_runs
    global ps
    global launched

    for i in range(n_runs):
        p = subprocess.Popen(
            ['./bin/esmini'] + list(sys.argv[1:]) +
            ['--param_permutation'] + [str(i)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL)
        ps.append([p, i])
        launched += 1
        print_status()

p = subprocess.run(['./bin/esmini', '--disable_stdout'] + list(sys.argv[1:]) + ['--return_nr_permutations'],
    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
n_runs = p.returncode

launch_thread = threading.Thread(target=launch_func)
launch_thread.start()
print_status()

while launched == 0 or len(ps) > 0:
    for p in ps:
        if (p[0].poll() is not None):
            print_status()
            ps.remove(p)
            done += 1
            print_status()
    time.sleep(0.5)

print('\n')