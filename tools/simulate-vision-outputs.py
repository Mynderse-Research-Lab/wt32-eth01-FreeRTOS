import random
import numpy
import sys
import math
import matplotlib.pyplot as plt
import time
from cell_net_l2 import CellNetL2Node, CellNodeId, UdpBridgeTransport, LinuxRawSocketL2Transport

#transport = LinuxRawSocketL2Transport(iface_name="enp3s0")
transport = UdpBridgeTransport(
    local_port=8851,
    remote_port=8850,
    remote_host="127.0.0.1",
)
node = CellNetL2Node(transport, node_id=CellNodeId.VISION)
node.begin()
item_id = 0
plt.ion()
fig,ax = plt.subplots(figsize=(6,4))
plt.show(block=False)

def main():
    global item_id
    #purpose: simulate 'random' battery placement, type, orientation, etc. for pick and place robot
    #need: physical limits of computer vision system, coordinate system for location defition
    #rotation, x1, y1, bounding box w and h

    #[cx, cy, w, h, angle] angle in radians 
    #can implement distribution informed by feedstock makeup?
    '''class id:
    1: 9V
    2: Pack of cylindrical cells
    3: Laptop 
    4: Power tool
    '''
    #classes = [[class name, class id, length[longest face] (mm), width (mm), height from belt (mm)], [class name, length, width], ...]
    classes = [["nine_volt_flatface", 1, 47.7, 26.6, 17.3],
            ["nine_volt_shortface", 1, 47.7, 17.3, 26.6],
            ["nine_volt_bottomface", 1, 26.6, 17.3, 47.7],
            ["pack", 2, 115.14, 68.3, 26.7],
            ["pack_shortface", 2, 115.14, 26.7, 68.3], #add distribution of batteries from feedstock?
            ["laptop", 3, 207.9, 50.3, 21.3],
            ["power_tool_makita", 4, 111.7, 73.8, 49.6],
            ["power_tool_makita_lenface", 4, 111.7, 49.6, 73.8],
            ["power_tool_makita_shortface", 4, 73.8, 49.6, 111.7],
            ["power_tool_NiCd_24V", 4, 148.4 ,92.1, 53],
            ["power_tool_NiCd_24V_shortface", 4, 92.1, 53, 148.4],
            ["power_tool_NiCd_24V_lenface", 4, 148.4, 53, 92.1],
            ["power_tool_rigid", 4, 117.8, 84.7, 43.3],
            ["power_tool_rigid_lenface", 4, 117.8, 43.3, 78.4],
            ["power_tool_rigid_shortface", 4, 78.4, 43.3, 117.8],
            ["power_tool_single_source", 4, 145.1, 78.7, 66.8],
            ["power_tool_single_source_lenface", 4, 145.1, 66.8, 78.7],
            ["power_tool_single_source_shortface", 4, 78.7, 66.8, 145.1],
            ["power_tool_gtv", 4, 124.3, 79.6, 53.2],
            ["power_tool_gtv_lenface", 4, 124.3, 53.2, 79.6],
            ["power_tool_gtv_shortface", 4, 79.6, 53.2, 124.3],
            ["power_tool_ryobi", 4, 97.2, 76.2, 54.1],
            ["power_tool_ryobi_lenface", 4, 97.2, 54.1, 76.2],
            ["power_tool_ryobi_shortface", 4, 76.2, 54.1, 97.2],
            ["power_tool_diehard", 4, 136.2, 77.4, 56.4],
            ["power_tool_diehard_lenface", 4, 136.2, 56.4, 77.4],
            ["power_tool_diehard_shortface", 4, 77.4, 56.4, 136.2]] #["cylindrical", ], ["power_tool"]] 


    #camera FOV: 14.5 in by 10.875 in
    #default resolution: 640 × 480 pixels
    #x-axis: || to gantry
    #y-axis: || to monitor

    x_max = 10.875 #in
    y_max = 14.5 #in

    x_max_mm = x_max * 25.4 #276.225
    y_max_mm = y_max * 25.4 #368.3 #how to put output string on network compatible wit c++ cellnet lib

    pot_angles = [numpy.pi/6, numpy.pi/4, numpy.pi/3, numpy.pi/2, 2*numpy.pi/3, 3*numpy.pi/4, 5*numpy.pi/6, numpy.pi]
    pot_angles_deg = numpy.arange(0, 360.25, 0.25).tolist()
    pot_angles_rad = numpy.deg2rad(pot_angles_deg)
    #print(pot_angles_deg)
    sim_class = random.choice(classes)
    print(sim_class[0])
    class_name = sim_class[0]
    class_id = sim_class[1]
    bb_length = sim_class[2]
    bb_width = sim_class[3]
    bb_height = sim_class[4]
    reset_flag = False

    bb_orientation = random.choice(pot_angles_rad) * random.choice([-1, 1])

    while not reset_flag:
        corner_coords = choose_center_coords(x_max_mm, y_max_mm, bb_length, bb_width, bb_orientation)
        #return [(x_c, y_c), (xp1, yp1), (xp2, yp2), (xp3, yp3), (xp4, yp4)]
        reset_flag = True
        for i in corner_coords[1:]:
            x,y = i
            if x < 0 or x > x_max_mm or y < 0 or y > y_max_mm:
                reset_flag = False
                
           
    x_data = [corner_coords[0][0], corner_coords[1][0], corner_coords[2][0], corner_coords[3][0], corner_coords[4][0]]
    y_data = [corner_coords[0][1], corner_coords[1][1], corner_coords[2][1], corner_coords[3][1], corner_coords[4][1]]
    item_id+=1

    print(f"(xc,yc): ({corner_coords[0][0]:.2f}, {corner_coords[0][1]:.2f})")
    print(f"(x1,y1): ({corner_coords[1][0]:.2f}, {corner_coords[1][1]:.2f})")
    print(f"(x2,y2): ({corner_coords[2][0]:.2f}, {corner_coords[2][1]:.2f})")
    print(f"(x3,y3): ({corner_coords[3][0]:.2f}, {corner_coords[3][1]:.2f})")
    print(f"(x4,y4): ({corner_coords[4][0]:.2f}, {corner_coords[4][1]:.2f})")
    print(f"angle: {bb_orientation} radians = {numpy.rad2deg(bb_orientation)} deg")
    
    output_string = f"xc: {corner_coords[0][0]} yc: {corner_coords[0][1]} width: {bb_width} length: {bb_length} orientation: {bb_orientation} class: {sim_class[0]}"
    print(output_string)
    sent = node.send_vision_detect(
        item_id = item_id,
        x_across_mm = float(corner_coords[0][0]),
        y_bat_mm = float(corner_coords[0][1]),
        theta_deg = float(numpy.rad2deg(bb_orientation)),
        battery_class = class_id, #expects int
        width_mm = float(bb_width),
        length_mm = float(bb_length),
        height_from_belt_mm = float(bb_height) 
    )
    '''
    fig, ax = plt.subplots(figsize=(6,4))
    ax.plot(y_data, x_data, color="black", marker="o", linestyle = "")
    ax.set_title("Vision Output")
    ax.set_xlim(0, y_max_mm)
    ax.set_ylim(0, x_max_mm)
    ax.plot([y_data[1], y_data[2]], [x_data[1],x_data[2]], color="black", linestyle="-")
    ax.plot([y_data[1], y_data[3]], [x_data[1],x_data[3]], color="black", linestyle="-")
    ax.plot([y_data[2], y_data[4]], [x_data[2],x_data[4]], color="black", linestyle="-")
    ax.plot([y_data[3], y_data[4]], [x_data[3],x_data[4]], color="black", linestyle="-")
    ax.set_aspect('equal')            
    plt.show()
    #plt.pause(5)
    #plt.close(fig)
    '''
    ax.clear()
    order=[1,2,4,3,1]
    ax.plot([corner_coords[j][1] for j in order],
            [corner_coords[j][0] for j in order],
            color="black",
            marker="o")
    ax.plot(corner_coords[0][1], corner_coords[0][0], color="black", marker="o")
    ax.set_title(f"Vison Output - Item {item_id}")
    ax.set_xlim(0,y_max_mm)
    ax.set_ylim(0, x_max_mm)
    ax.set_aspect("equal")
    fig.canvas.draw_idle()

    

def choose_center_coords(x_max_mm, y_max_mm, length, width, theta):
    x_c = random.randint(0, int(x_max_mm))
    y_c = random.randint(0,int(y_max_mm))
    #orientation: 0 deg rotation is longest face parallel to x axis
    #theta is angle (rad) between battery longest face and x axis

    #bb_orientation = random.choice(pot_angles) * random.choice([-1, 1])

    #angle_to_corner = numpy.arctan((width/2)/(length/2)) #tan(0.5w/0.5h) = angle

    xp1 = x_c - ((length/2) * numpy.cos(theta)) + ((width/2) * numpy.sin(theta))
    yp1 = y_c - ((length/2) * numpy.sin(theta)) - ((width/2) * numpy.cos(theta))
    xp2 = x_c - ((length/2) * numpy.cos(theta)) - ((width/2) * numpy.sin(theta))
    yp2 = y_c - ((length/2) * numpy.sin(theta)) + ((width/2) * numpy.cos(theta))
    xp3 = x_c + ((length/2) * numpy.cos(theta)) + ((width/2) * numpy.sin(theta))
    yp3 = y_c + ((length/2) * numpy.sin(theta)) - ((width/2) * numpy.cos(theta))
    xp4 = x_c + ((length/2) * numpy.cos(theta)) - ((width/2) * numpy.sin(theta))
    yp4 = y_c + ((length/2) * numpy.sin(theta)) + ((width/2) * numpy.cos(theta))

    return [[x_c, y_c], [xp1, yp1], [xp2, yp2], [xp3, yp3], [xp4, yp4]]

'''
while(1):
    #plt.close('all')
    main()
    time.sleep(5)
'''

frame_interval = 0.5 #10 detections per sec?
try:
    while plt.fignum_exists(fig.number):
        start = time.perf_counter()
        main()
        elapsed = time.perf_counter() - start
        plt.pause(max(0.001, frame_interval - elapsed))
except KeyboardInterrupt:
    pass
finally:
    transport.close()
    plt.close(fig)