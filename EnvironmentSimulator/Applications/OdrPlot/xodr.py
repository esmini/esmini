
import sys
import csv
import math
import matplotlib.pyplot as plt

H_SCALE = 10

with open(sys.argv[1]) as f:
    reader = csv.reader(f, skipinitialspace=True)
    positions = list(reader)

ref_x = []
ref_y = []
ref_z = []
ref_h = []

lane_x = []
lane_y = []
lane_z = []
lane_h = []

ref = False
for pos in positions:
    if pos[0] == 'lane':
        if pos[3] == '0':
            ref = True
            ref_x.append([])
            ref_y.append([])
            ref_z.append([])
            ref_h.append([])
        else:
            ref = False
            lane_x.append([])
            lane_y.append([])
            lane_z.append([])
            lane_h.append([])
    else:
        if ref:
            ref_x[-1].append(float(pos[0]))
            ref_y[-1].append(float(pos[1]))
            ref_z[-1].append(float(pos[2]))
            ref_h[-1].append(float(pos[3]) + math.pi/2.0)
        else:
            lane_x[-1].append(float(pos[0]))
            lane_y[-1].append(float(pos[1]))
            lane_z[-1].append(float(pos[2]))
            lane_h[-1].append(float(pos[3]) + math.pi / 2.0)

p1 = plt.figure(1)
for i in range(len(ref_x)):
    plt.plot(ref_x[i], ref_y[i], linewidth=2.0, color='#BB5555')
for i in range(len(lane_x)):
    plt.plot(lane_x[i], lane_y[i], linewidth=1.0, color='#3333BB')

# hdg_lines = []
# for i in range(len(h)):
#     for j in range(len(h[i])):
#         hx = x[i][j] + H_SCALE * math.cos(h[i][j])
#         hy = y[i][j] + H_SCALE * math.sin(h[i][j])
#         plt.plot([x[i][j], hx], [y[i][j], hy])


p1.gca().set_aspect('equal', adjustable='box')

#p2 = plt.figure(2)
#for i in range(len(z)):
#    ivec = [j for j in range(len(z[i]))]
#    plt.plot(z[i])

#plt.gca().set_aspect('equal', adjustable='box')
plt.show()


