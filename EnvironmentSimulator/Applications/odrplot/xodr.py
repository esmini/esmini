
import sys
import csv
import math
import matplotlib.pyplot as plt

H_SCALE = 10
text_x_offset = 0
text_y_offset = 0.7
text_size = 7

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

border_x = []
border_y = []
border_z = []
border_h = []

road_id = []
road_id_x = []
road_id_y = []

road_start_dots_x = []
road_start_dots_y = []

road_end_dots_x = []
road_end_dots_y = []

lane_section_dots_x = []
lane_section_dots_y = []

arrow_dx = []
arrow_dy = []

current_road_id = None
current_lane_id = None
current_lane_section = None
new_lane_section = False

for i in range(len(positions) + 1):
    
    if i < len(positions):
        pos = positions[i]
    
    # plot road id before going to next road
    if i == len(positions) or (pos[0] == 'lane' and i > 0 and current_lane_id == '0'):

        if current_lane_section == '0':
            road_id.append(int(current_road_id))
            index = int(len(ref_x[-1])/3.0)
            h = ref_h[-1][index]
            road_id_x.append(ref_x[-1][index] + (text_x_offset * math.cos(h) - text_y_offset * math.sin(h)))
            road_id_y.append(ref_y[-1][index] + (text_x_offset * math.sin(h) + text_y_offset * math.cos(h)))
            road_start_dots_x.append(ref_x[-1][0])
            road_start_dots_y.append(ref_y[-1][0])
            if len(ref_x) > 0:
                arrow_dx.append(ref_x[-1][1]-ref_x[-1][0])
                arrow_dy.append(ref_y[-1][1]-ref_y[-1][0])
            else:
                arrow_dx.append(0)
                arrow_dy.append(0)

        lane_section_dots_x.append(ref_x[-1][-1])
        lane_section_dots_y.append(ref_y[-1][-1])
    
    if i == len(positions):
        break

    if pos[0] == 'lane':
        current_road_id = pos[1]
        current_lane_section = pos[2]
        current_lane_id = pos[3]
        if pos[3] == '0':
            ltype = 'ref'
            ref_x.append([])
            ref_y.append([])
            ref_z.append([])
            ref_h.append([])

        elif pos[4] == 'no-driving':
            ltype = 'border'
            border_x.append([])
            border_y.append([])
            border_z.append([])
            border_h.append([])
        else:
            ltype = 'lane'
            lane_x.append([])
            lane_y.append([])
            lane_z.append([])
            lane_h.append([])
    else:
        if ltype == 'ref':
            ref_x[-1].append(float(pos[0]))
            ref_y[-1].append(float(pos[1]))
            ref_z[-1].append(float(pos[2]))
            ref_h[-1].append(float(pos[3]))
                
        elif ltype == 'border':
            border_x[-1].append(float(pos[0]))
            border_y[-1].append(float(pos[1]))
            border_z[-1].append(float(pos[2]))
            border_h[-1].append(float(pos[3]))
        else:
            lane_x[-1].append(float(pos[0]))
            lane_y[-1].append(float(pos[1]))
            lane_z[-1].append(float(pos[2]))
            lane_h[-1].append(float(pos[3]))

p1 = plt.figure(1)

# plot road ref line segments
for i in range(len(ref_x)):
    plt.plot(ref_x[i], ref_y[i], linewidth=2.0, color='#BB5555')

# plot driving lanes in blue
for i in range(len(lane_x)):
    plt.plot(lane_x[i], lane_y[i], linewidth=1.0, color='#3333BB')
    
# plot border lanes in gray
for i in range(len(border_x)):
    plt.plot(border_x[i], border_y[i], linewidth=1.0, color='#AAAAAA')

# plot red dots indicating lane dections
for i in range(len(lane_section_dots_x)):
    plt.plot(lane_section_dots_x[i], lane_section_dots_y[i], 'o', ms=4.0, color='#BB5555')

for i in range(len(road_start_dots_x)):
    # plot a yellow dot at start of each road
    plt.plot(road_start_dots_x[i], road_start_dots_y[i], 'o', ms=5.0, color='#BBBB33')
    # and an arrow indicating road direction
    plt.arrow(road_start_dots_x[i], road_start_dots_y[i], arrow_dx[i], arrow_dy[i], width=0.1, head_width=1.0, color='#BB5555')

# plot road id numbers
for i in range(len(road_id)):
    plt.text(road_id_x[i], road_id_y[i], road_id[i], size=text_size, ha='center', va='center', color='#222222')


plt.gca().set_aspect('equal', 'datalim')

#p2 = plt.figure(2)
#for i in range(len(z)):
#    ivec = [j for j in range(len(z[i]))]
#    plt.plot(z[i])

plt.show()


