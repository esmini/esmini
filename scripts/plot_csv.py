import matplotlib.pyplot as plt
import argparse

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

file = open(args.filename, 'r')

print(file.readline().strip())

param_names = file.readline().strip().split(', ')

print('Available parameters:', ', '.join(param_names))

if args.param is None:
    exit(0)

n_parameters = len(args.param)
parameter = []

for a in args.param:
    parameter.append(a)

x_index = -1
y_index = []
objs = []
x = []
y = []

if args.x_axis is None:
    x_axis = param_names[0]  # default x_axis == time
else:
    x_axis = args.x_axis.strip()

print('x_axis:', x_axis)
print('parameters:', ', '.join(parameter))

for i, p in enumerate(param_names):
    p = p.strip()

    if p == x_axis:
        x_index = i

for i, p in enumerate(parameter):
    for j, a in enumerate(param_names):
        a = a.strip()
        if a == p:
            y_index.append(j)
    if len(y_index) != i+1:
        print('Parameter {} not found'.format(p))
        exit(-1)

if x_index == -1:
    print("x_axis {} not found".format(args.x_axis.strip()))
    exit(-1)

while True:

    # Get next line from file
    line = file.readline()
    data = line.strip().split(', ')

    # if line is empty end of file is reached
    if not line:
        break

    i = int(data[1])
    if i >= len(objs):
        print('adding obj {} {}'.format(i, data[2]))
        objs.append(data[2])
        x.append([])
        y.append([])
        for p in y_index:
            y[i].append([])

    for j, p in enumerate(y_index):
        y[i][j].append(float(data[p]))
    x[i].append(float(data[x_index]))

file.close()

p1 = plt.figure(1)

for i in range(len(x)):
    for j in range(len(y[i])):
        plt.plot(x[i], y[i][j], linewidth=1.0, label=objs[i] + ' ' + parameter[j])

plt.xlabel(x_axis)
plt.legend(loc="upper right")

if args.equal_axis_aspect:
    plt.gca().set_aspect('equal', 'datalim')

plt.show()
