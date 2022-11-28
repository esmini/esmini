import argparse
import numpy as np
import sys

import plot

if __name__ == "__main__":
    # Create the parser
    parser = argparse.ArgumentParser(description='Plot esmini log data')

    # Add the arguments
    parser.add_argument('filename', help='dat filename')
    parser.add_argument('--x_axis', help='x-axis parameter', default='time')
    parser.add_argument('--equal_axis_aspect', help='lock aspect ratio = 1:1', action='store_true')
    parser.add_argument('--derive', help='derive values wrt x, i.e. dy/dx', action='store_true')
    parser.add_argument('--dots', help='add dots', action='store_true')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--list_params', help='list available parameters in given file', action='store_true')
    group.add_argument('--param', help='parameter(s) to plot. Multiple parameters separated by "," NOTE: no space!', action='append')

    # Execute the parse_args() method
    args = parser.parse_args()

    # Read the dat file
    data = np.genfromtxt(sys.argv[1], delimiter=',', names=True, dtype=None, encoding=None, autostrip=True)

    rows = []
    for r in data:
        rows.append(r)

    plot.plot(
        rows, data.dtype.names,
        args.param,
        args.x_axis,
        args.derive,
        args.dots,
        args.equal_axis_aspect,
        args.list_params
    )
