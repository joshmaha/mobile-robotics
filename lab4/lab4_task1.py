# import MyRobot Class and math library
from fairis_tools.my_robot import MyRobot
import math
from scipy.stats import norm
from matplotlib import pyplot as plt

# create robot instance
josh = MyRobot()

# load environment from maze file
maze_file = '../../worlds/Fall24/maze7.xml'
josh.load_environment(maze_file)

# Move robot to starting position
josh.move_to_start()
landmarks = {
1: (-2.5, -2.5), # Green cylinder
2: (-2.5, 2.5), # Yellow cylinder
3: (2.5, 2.5), # Red cylinder
4: (2.5, -2.5) # Blue cylinder
}

# Predefined centers for each cell in a 5x5 grid
cell_centers = {
1: (-2, 2), 2: (-1, 2), 3: (0, 2), 4: (1, 2),
5: (2, 2),
6: (-2, 1), 7: (-1, 1), 8: (0, 1), 9: (1, 1),
10: (2, 1),
11: (-2, 0), 12: (-1, 0), 13: (0, 0), 14: (1, 0),
15: (2, 0),
16: (-2, -1), 17: (-1, -1), 18: (0, -1), 19: (1, -1),
20: (2, -1),
21: (-2, -2), 22: (-1, -2), 23: (0, -2), 24: (1, -2),
25: (2, -2)
}

# gps points and encoder points to plot graph
x_gps_pts = []
y_gps_pts = []
x_estimated_pts = [-2,-1, 0, 1, 2, -2, -1, 0, 1, 2, -2, -1, 0, 1, 2, -
2, -1, 0, 1, 2, -2, -1, 0, 1, 2]
y_estimated_pts = [ 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, -
1, -1, -1, -1, -1, -2, -2, -2, -2, -2]

# Precomputed distances stored in a dictionary
precomputed_distances = {}
for cell, center_coords in cell_centers.items(): # Iterate over each cell
    distances = {}
for lm, lm_coords in landmarks.items(): # Calculate distance to each landmark
    distances[lm] = math.sqrt((center_coords[0] - lm_coords[0])**2 + (center_coords[1] - lm_coords[1])**2)
precomputed_distances[cell] = distances
# # Print the full dictionary in a structured format
# for cell, distances in precomputed_distances.items():
# print(f"Cell {cell}:")
# for lm, dist in distances.items():
# print(f" Landmark {lm}: {dist:.2f}")
# print()
###################################################################################
######################

# Measured distances using Lidar, IMU and RGB sensors
def get_cell(measured_dist_l1, measured_dist_l2, measured_dist_l3, measured_dist_l4):
    sigma = 1 # Standard deviation for measurement noise
    probabilities = {}
    # Iterate through all cells
    for cell, distances in precomputed_distances.items():
        # Precomputed distances for the current cell
        precomputed_l1 = distances[1]
        precomputed_l2 = distances[2]
        precomputed_l3 = distances[3]
        precomputed_l4 = distances[4]
        # Calculate probabilities for each landmark using Gaussian PDF
        prob_l1 = norm.pdf(measured_dist_l1, precomputed_l1, sigma)
        prob_l2 = norm.pdf(measured_dist_l2, precomputed_l2, sigma)
        prob_l3 = norm.pdf(measured_dist_l3, precomputed_l3, sigma)
        prob_l4 = norm.pdf(measured_dist_l4, precomputed_l4, sigma)

        # Combine probabilities (e.g., by multiplying them)
        combined_prob = prob_l1 * prob_l2 * prob_l3 * prob_l4
        probabilities[cell] = combined_prob
    # Find the cell with the highest probability
    best_cell = max(probabilities, key=probabilities.get)
    print(f"Best estimated cell: {best_cell} with probability {probabilities[best_cell]:.4f}")
    lidar_fd = josh.get_lidar_range_image()[400]
    lidar_right = josh.get_lidar_range_image()[600]
    lidar_left = josh.get_lidar_range_image()[200]
    lidar_rear = josh.get_lidar_range_image()[0]
    #print(f"Distance from wall in front:{lidar_fd:.2f} right:{lidar_right:.2f} left:{lidar_left:.2f} rear:{lidar_rear:.2f}")
    #return best_cell
    if josh.get_compass_reading() in range(0, 359):
        while josh.experiment_supervisor.step(josh.timestep) != -1:
            if josh.get_compass_reading() not in range(89, 91):
                josh.rotate_in_place(2)
        else:
            josh.stop()
            return best_cell
# make robot face north
def rotate_until_90():
    compass = josh.get_compass_reading()
    josh.turn_in_place(45, "L")
    #print(compass)

x,y,prev_left_distance, prev_right_distance = 0,0,0,0

# update robot x, y and theta coordinates
def pose_update():
    global x, y, prev_left_distance, prev_right_distance
    # get encoder readings (in meters)
    curr_front_left_d = josh.get_front_left_motor_encoder_reading() * josh.wheel_radius
    curr_rear_left_d = josh.get_rear_left_motor_encoder_reading() *    josh.wheel_radius
    curr_front_right_d = josh.get_front_right_motor_encoder_reading() *    josh.wheel_radius
    curr_rear_right_d = josh.get_rear_right_motor_encoder_reading() *    josh.wheel_radius
   
    # average how much left and right wheels have moved since prior update
    delta_left = ((curr_front_left_d + curr_rear_left_d)/2) - prev_left_distance
    delta_right = ((curr_front_right_d + curr_rear_right_d)/2) -    prev_right_distance
   
    # average the wheel movements to estimate the total distance traveled
    delta_d = (delta_left + delta_right) / 2
   
    # get the current compass reading in degrees and convert to radians
    compass_degrees = josh.get_compass_reading()
    theta = josh.radians(compass_degrees) # Update heading angle
   
    # find change in x and y based on the movement
    delta_x = delta_d * math.cos(theta)
    delta_y = delta_d * math.sin(theta)
   
    # update x and y positions
    x += delta_x
    y += delta_y
   
    # update previous encoder values
    prev_left_distance = ((curr_front_left_d + curr_rear_left_d)/2)
    prev_right_distance = ((curr_front_right_d + curr_rear_right_d)/2)
   
    # output updated position
    #print(f"Current Pose: x = {x:.2f}, y = {y:.2f}, theta = {math.degrees(theta):.2f}°")
    
    # plot gps graph of x and y points
    gps_values = josh.gps.getValues()
    x_pt = gps_values[0]
    y_pt = gps_values[1]
    x_gps_pts.append(x_pt)
    y_gps_pts.append(y_pt)

#plot graph
def graph_plotter():
    # plotting both graphs
    plt.figure(1)
    plt.plot(x_gps_pts, y_gps_pts,label='GPS Position',marker='o',
    linestyle='dotted', color='b')
    plt.plot(x_estimated_pts, y_estimated_pts,label='Estimated    Position',marker='*', linestyle='dashed', color='r')
    plt.title("GPS vs Estimated Position")
    plt.xlabel("X(m)")
    plt.ylabel("Y(m)")
    plt.legend()
    plt.grid(True)
    plt.show()

###################################################################################
######################
# traverse maze in lawnmower patter, go to far left, go up to top left corner
# then start lawnmower pattern going from left to right then down one cell until cell 25

# set to track visited cells
visited_cells = set()

# def navigate_maze(best_cell):
    # global visited_cells
    # visited_cells.add(best_cell) #add best_cell to found cells set
    # while len(visited_cells) < 25: # Continue until all cells are visited
    # # Get wall distances
    # lidar_fd = josh.get_lidar_range_image()[400]
    # lidar_right = josh.get_lidar_range_image()[600]
    # lidar_left = josh.get_lidar_range_image()[200]
    # lidar_rear = josh.get_lidar_range_image()[0]
    # #print(f"LiDAR distances: Forward: {lidar_fd:.2f}, Right:{lidar_right:.2f}, Left: {lidar_left:.2f}, Rear: {lidar_rear:.2f}")
    # if lidar_fd > 0.5: # go forward if there's no wall in front
    # josh.straight_move(0, 1)
    # josh.stop()
    # #jump back to scan for objects and best cell calculation
    # break
    # # elif lidar_right > 0.5: # turn right if there's space
    # # josh.straight_move(0, 1)
    # # josh.stop()
    # # print("Turned and moved right 1m.")
    # # elif lidar_left > 0.5: # turn left if there's space
    # # josh.turn_in_place(5)
    # # josh.straight_move(0, 1)
    # # josh.stop()
    # # print("Turned and moved left 1m.")
    # # else: # Dead-end, turn around
    # # josh.straight_move(0, 1)
    # # josh.stop()
    # # print("Turned and moved back 1m.")
    # print(f"Visited cells: {visited_cells}")
    # print("All cells visited. Maze navigation complete.")

l1_exists, l2_exists, l3_exists, l4_exists= 0,0,0,0
landmark_count = 0
measured_dist_l1, measured_dist_l2, measured_dist_l3, measured_dist_l4 = 0,0,0,0

while josh.experiment_supervisor.step(josh.timestep) != -1:
    josh.rotate_in_place(3)
    landmark_distances = []
    rec_objects = josh.rgb_camera.getRecognitionObjects()
    if len(rec_objects) > 0:
        l1 = rec_objects[0]
        r1,g1,b1 = l1.getColors()[0], l1.getColors()[1], l1.getColors()[2]
        x_on_image, y_on_image = l1.getPositionOnImage()[0], l1.getPositionOnImage()[1]
    
        # check for L1 which is the green landmark
        if(r1==0 and g1==1 and b1==0):
            l1_x_pos, l1_y_pos, l1_z_pos = l1.getPosition()[0], l1.getPosition()[1], l1.getPosition()[2]
            #print(f" Green distance x:{l1_x_pos:.2f} y:{l1_y_pos:.2f} z:{l1_z_pos:.2f}")
            if(x_on_image in range(310,340) and y_on_image in range(150,175)):
                measured_dist_l1 = l1_x_pos
                #print(f"Robot measured distance from green landmark:{measured_dist_l1:.2f}")
                l1_exists = 1
                #landmark_count += 1
    
        # check for L2 which is the yellow landmark
        if(r1==1 and g1==1 and b1==0):
            l2_x_pos, l2_y_pos, l2_z_pos = l1.getPosition()[0], l1.getPosition()[1], l1.getPosition()[2]
            #print(f"Yellow distance x:{l2_x_pos:.2f} y:{l2_y_pos:.2f} z:{l2_z_pos:.2f}")
            if(x_on_image in range(310,340) and y_on_image in range(150,175)):
                #josh.stop()
                measured_dist_l2 = l2_x_pos
                #print(f"Robot measured distance from yellow landmark:{measured_dist_l2:.2f}")
                l2_exists = 1
                #landmark_count += 1

        # check for L3 which is the red landmark
        if(r1==1 and g1==0 and b1==0):
            l3_x_pos, l3_y_pos, l3_z_pos = l1.getPosition()[0], l1.getPosition()
            [1], l1.getPosition()[2]
            #print(f"Red distance x:{l3_x_pos:.2f} y:{l3_y_pos:.2f} z:{l3_z_pos:.2f}")
            if(x_on_image in range(310,340) and y_on_image in range(150,175)):
                #josh.stop()
                measured_dist_l3 = l3_x_pos
                #print(f"Robot measured distance from red landmark: {measured_dist_l3:.2f}")
                l3_exists = 1
                #landmark_count += 1
    
        # check for L4 which is the blue landmark
        if(r1==0 and g1==0 and b1==1):
            l4_x_pos, l4_y_pos, l4_z_pos = l1.getPosition()[0], l1.getPosition()[1], l1.getPosition()[2]
            #print(f"Blue distance x:{l4_x_pos:.2f} y:{l4_y_pos:.2f} z:{l4_z_pos:.2f}")
            if(x_on_image in range(310,340) and y_on_image in range(150,175)):
                #josh.stop()
                measured_dist_l4 = l4_x_pos
                #print(f"Robot measured distance from blue landmark:{measured_dist_l4:.2f}")
                l4_exists = 1
                #landmark_count += 1

        if (l1_exists and l2_exists and l3_exists and l4_exists):
            josh.stop()
        # print(f"Measured distance from green landmark 1:{measured_dist_l1:.2f}")
        # print(f"Measured distance from yellow landmark 2:{measured_dist_l2:.2f}")
        # print(f"Measured distance from red landmark 3:{measured_dist_l3:.2f}")
        # print(f"Measured distance from blue landmark 4:{measured_dist_l4:.2f}")
        current_cell = get_cell(measured_dist_l1, measured_dist_l2, measured_dist_l3, measured_dist_l4)
        #navigate_maze(current_cell) #not working
        #compass = josh.get_compass_reading()
        #print(compass)
        # while compass != 90:
        #   josh.rotate_in_place(2)
        #   #josh.stop()
        #   if compass == 90:
        #       josh.stop()
        #       break
        # #rotate_until_90()
        # josh.stop()
        #while()
        break
        #go to next phase of program

graph_plotter()
