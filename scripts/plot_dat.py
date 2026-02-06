import argparse
import os
import sys
import subprocess
from typing import List
import numpy as np

SCRIPT_FOLDER = os.path.dirname(os.path.realpath(__file__))
sys.path.append(SCRIPT_FOLDER)  # add script folder to path to find dependent modules
import plot

ESMINI_PATH = os.path.realpath(os.path.join(SCRIPT_FOLDER, '..'))  # path to esmini root folder

def get_labels_line_extended() -> List[str]:
    """ Get the extended labels line """
    return ['time', 'id', 'name', 'x', 'y', 'z', 'h', 'p', 'r', 'roadId', 'laneId', 'offset', 't', 's', 'speed', 'wheel_angle', 'wheel_rot']

if __name__ == "__main__":
    # Create the parser
    parser = argparse.ArgumentParser(description='Plot esmini log data')

    # Add the arguments
    parser.add_argument('--x_axis', help='x-axis parameter', default='time')
    parser.add_argument('--equal_axis_aspect', help='lock aspect ratio = 1:1', action='store_true')
    parser.add_argument('filename', help='dat filename')
    parser.add_argument('--derive', help='derive values wrt x, i.e. dy/dx', action='store_true')
    parser.add_argument('--dots', help='add dots', action='store_true')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--list_params', help='list available parameters in given file', action='store_true')
    group.add_argument('--param', help='parameter(s) to plot. Multiple parameters separated by "," NOTE: no space!', action='append')

    # Execute the parse_args() method
    args = parser.parse_args()

    esmini_args = [os.path.join(ESMINI_PATH,'bin','dat2csv'), "--file", args.filename, '--extended', '--print_csv']
    process = subprocess.Popen(esmini_args, stdout=subprocess.PIPE, text=True)
    csv_data = process.stdout.read().strip().splitlines()

    data = np.genfromtxt(csv_data[1:], delimiter=',', names=True, dtype=None, encoding=None, autostrip=True)

    plot.plot(
        data, get_labels_line_extended(),
        args.param,
        args.x_axis,
        args.derive,
        args.dots,
        args.equal_axis_aspect,
        args.list_params
    )
