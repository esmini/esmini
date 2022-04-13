from dat import *

import matplotlib.pyplot as plt

if __name__ == "__main__":
    # Create the parser
    parser = argparse.ArgumentParser(description='Plot esmini log data')

    # Add the arguments
    parser.add_argument('--x_axis', help='x-axis parameter')
    parser.add_argument('--equal_axis_aspect', help='lock aspect ratio = 1:1', action='store_true')
    parser.add_argument('filename', help='csv filename')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--list_params', help='list available parameters in given file', action='store_true')
    group.add_argument('--param', help='parameter to plot (can be specified multiple times)', action='append')

    # Execute the parse_args() method
    args = parser.parse_args()

    # Read the dat file
    dat = DATFile(args.filename)

    plottable_params = []
    for label in dat.labels:
        if type(getattr(dat.data[0], label)) == float or type(getattr(dat.data[0], label)) == int:
            plottable_params.append(label)

    if args.list_params or args.param is None:
        parser.print_usage()
        print('\nPlottable parameters:')
        print (*plottable_params, sep=', ')
        exit(0)

    n_parameters = len(args.param)
    parameter = []
    for a in args.param:
        if a not in plottable_params:
            if a in dat.labels:
                print('Parameter \'{}\' is not plottable'.format(a))
            else:
                print('Parameter \'{}\' is not available in dat file'.format(a))
            exit(0)
        parameter.append(a)

    objs = []
    x = []
    y = []
    id2idx = {}

    if args.x_axis is None:
        x_axis = 'time'
    else:
        x_axis = args.x_axis.strip()

    print('x_axis:', x_axis)
    print('parameters:', ', '.join(parameter))

    for data in dat.data:
        id = int(data.id)
        if id not in id2idx:
            print('adding object {} {}'.format(id, data.name.decode('utf-8')))
            id2idx[id] = len(objs)
            objs.append(data.name.decode('utf-8'))
            x.append([])
            y.append([])
            for p in parameter:
                y[id2idx[id]].append([])

        for j, p in enumerate(parameter):
            y[id2idx[id]][j].append(float(getattr(data, p)))
        x[id2idx[id]].append(float(getattr(data, x_axis)))

    p1 = plt.figure(1)
    for i in range(len(x)):
        for j, p in enumerate(parameter):
            plt.plot(x[i], y[i][j], linewidth=1.0, label=objs[i] + ' ' + p)

    plt.xlabel(x_axis)
    plt.legend(loc="upper right")

    if args.equal_axis_aspect:
        plt.gca().set_aspect('equal', 'datalim')

    plt.show()