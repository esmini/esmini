import matplotlib.pyplot as plt
import numpy as np

def plot(rows, labels, params=['speed'], x_axis='time', derive=False, dots=False, equal_aspect=False, list_plottable_params=False):

    if not x_axis in labels:
        raise RuntimeError('x_axis: Did not find {} in data labels {}'.format(x_axis, labels))

    x_axis_index = labels.index(x_axis)

    if params is not None:
        params_complete = []
        for p in params:
            params_complete += p.split(",")

        params = params_complete

    print('x_axis:', x_axis)
    print('parameters:', params)

    plottable_params = []
    for i, label in enumerate(labels):
        value = rows[0][i]
        if isinstance(value, float) or isinstance(value, int):
            plottable_params.append(label.strip())

    if list_plottable_params or params is None:
        print('\nPlottable parameters:')
        print (*plottable_params, sep=', ')
        return

    parameter = []
    for a in params:
        if a not in plottable_params:
            if a in labels:
                print('Parameter \'{}\' is not plottable'.format(a))
            else:
                print('Parameter \'{}\' is not available in data'.format(a))
            return
        parameter.append(a)

    # filter out requested parameters and slice out objects
    objs = []
    x = []
    y = []
    id2idx = {}

    if not 'id' in labels:
        raise RuntimeError('Did not find id in labels {}'.format(labels))
    id_index = labels.index('id')

    for data in rows:
        id = data[id_index]
        if id not in id2idx:
            if 'name' in labels:
                name = data[labels.index('name')].strip()
            else:
                name = 'obj' + str(id)
            name += ' ({})'.format(id)
            id2idx[id] = len(objs)
            objs.append(name)
            x.append([])
            y.append([])
            for p in parameter:
                y[id2idx[id]].append([])

        for j, p in enumerate(parameter):
            if p in params:
                y[id2idx[id]][j].append(float(data[labels.index(p)]))
            else:
                 raise RuntimeError('Did not find param {} in labels {}'.format(p, labels))
        x[id2idx[id]].append(float(data[x_axis_index]))

    if derive:
        for i in range(len(y)):
            for j in range(len(y[i])):
                new_col = []
                for k in range(1,len(y[i][j])):
                    value_prim = (y[i][j][k] - y[i][j][k-1]) / max((x[j][k] - x[j][k-1], 1e-10))
                    new_col.append(value_prim)
                new_col.append(new_col[-1])  # duplicate last entry
                y[i][j] = new_col

    p1 = plt.figure(1)
    for i in range(len(x)):
        for j, p in enumerate(params):
            if dots:
                p_style = '.-'
            else:
                p_style = '-'
            plt.plot(x[i], y[i][j], p_style, linewidth=1.0, label=objs[i] + ' ' + p)

    plt.grid(True, color='whitesmoke')
    plt.xlabel(x_axis)
    plt.legend(loc="upper right")

    if equal_aspect:
        plt.gca().set_aspect('equal', 'datalim')

    plt.show()

if __name__ == "__main__":
    print('This is a sub package. Use plot_*.py scripts according to file type.')