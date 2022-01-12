import sys
import os

esmini_path = os.getenv('ESMINI_PATH')
if esmini_path is None:
    esmini_path = os.path.join('.', 'bin', 'esmini')

if (len(sys.argv) < 2):
    print('Usage: {} <xosc file>'.format(sys.argv[0]))
    exit(-1)

os.system(esmini_path + ' --window 60 60 800 400' + ' --osc ' + sys.argv[1])
