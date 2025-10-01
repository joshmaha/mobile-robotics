# Task 2
# Import MyRobot Class and math library
from fairis_tools.my_robot import MyRobot
import math
from scipy.stats import norm
from matplotlib import pyplot as plt

# Create robot instance
josh = MyRobot()

# Load environment from maze file
maze_file = '../../worlds/Fall24/maze8.xml'
josh.load_environment(maze_file)

# Move robot to starting position
josh.move_to_start()
tot_dist = 0

# Data structure to store each cell wall configuration data
# 1 indicates wall; 0 indicates no wall
# WNES - West, North, East, South
maze_cells = {
1: {"W": 1, "N": 1, "E": 0, "S": 0}, #WWOO
2: {"W": 0, "N": 1, "E": 0, "S": 1}, #OWOW
3: {"W": 0, "N": 1, "E": 0, "S": 1}, #OWOW
4: {"W": 0, "N": 1, "E": 0, "S": 1}, #OWOW
5: {"W": 0, "N": 1, "E": 1, "S": 0}, #OWWO
6: {"W": 1, "N": 0, "E": 1, "S": 0}, #WOWO
7: {"W": 1, "N": 1, "E": 0, "S": 1}, #WWOW
8: {"W": 0, "N": 1, "E": 0, "S": 1}, #OWOW
9: {"W": 0, "N": 1, "E": 0, "S": 1}, #OWOW
10: {"W": 0, "N": 0, "E": 1, "S": 1},#OOWW
11: {"W": 1, "N": 0, "E": 0, "S": 0},#WOOO
12: {"W": 0, "N": 1, "E": 0, "S": 1},#OWOW
13: {"W": 0, "N": 1, "E": 0, "S": 1},#OWOW
14: {"W": 0, "N": 1, "E": 0, "S": 1},#OWOW
15: {"W": 0, "N": 1, "E": 0, "S": 0},#OWOO
16: {"W": 1, "N": 0, "E": 0, "S": 0},#WOOO
17: {"W": 0, "N": 1, "E": 0, "S": 1},#OWOW
18: {"W": 0, "N": 1, "E": 0, "S": 1},#OWOW
19: {"W": 0, "N": 1, "E": 0, "S": 1},#OWOW
20: {"W": 0, "N": 0, "E": 1, "S": 0},#OOWO
21: {"W": 1, "N": 0, "E": 0, "S": 1},#WOOW
22: {"W": 0, "N": 1, "E": 0, "S": 1},#OWOW
23: {"W": 0, "N": 1, "E": 0, "S": 1},#OWOW
24: {"W": 0, "N": 1, "E": 0, "S": 1},#OWOW
25: {"W": 0, "N": 0, "E": 1, "S": 1},#OOWW
}

# attempted to use this to check bordering cells and enter new cell in a systematic manner
# maze connectivity (manual adjacency list)
maze_connections = {
1: [2, 6], 2: [1, 3], 3: [2, 4], 4: [3, 5], 5: [4],
6: [1, 11], 7: [8], 8: [7, 9], 9: [8, 10], 10: [9],
11: [6, 12], 12: [11, 13], 13: [12, 14], 14: [13, 15], 15: [14, 20],
16: [17, 21], 17: [16, 18], 18: [17, 19], 19: [18, 20], 20: [15,19],
21: [16, 22], 22: [21, 23], 23: [22, 24], 24: [23, 25], 25: [24],
}

# Probability method of comparing lidar readings to precalculated wall configuration
def get_probability(lidar_walls, expected):
    if lidar_walls == expected:
        return 0.8 if lidar_walls == 1 else 0.6
    else:
        return 0.2 if lidar_walls == 1 else 0.4

# Calculate likelihood for a single cell
def calculate_cell_probability(lidar_wall_detected, expected_walls):
    north_prob = get_probability(lidar_wall_detected["N"], expected_walls["N"])
    south_prob = get_probability(lidar_wall_detected["S"], expected_walls["S"])
    east_prob = get_probability(lidar_wall_detected["E"], expected_walls["E"])
    west_prob = get_probability(lidar_wall_detected["W"], expected_walls["W"])
    return north_prob * south_prob * east_prob * west_prob

# Compute prob_vals for all cells
def localize_robot(maze, lidar_wall_detected):
    prob_vals = {}
    for cell, expected_walls in maze.items():
        prob_vals[cell] = round(calculate_cell_probability(lidar_wall_detected, expected_walls), 2)
    return prob_vals

# Get wall distances using LiDAR
def get_wall_dist():
    north_lidar = josh.get_lidar_range_image()[400]
    east_lidar = josh.get_lidar_range_image()[600]
    west_lidar = josh.get_lidar_range_image()[200]
    south_lidar = josh.get_lidar_range_image()[0]

    # Variable to see if LiDAR picks up wall reading
    north_wall, west_wall, south_wall, east_wall = 0, 0, 0, 0
    print(f"Distance West: {west_lidar:.2f}m North: {north_lidar:.2f}m East:{east_lidar:.2f}m South: {south_lidar:.2f}m")
    if west_lidar < 0.51:
        west_wall = 1
    if north_lidar < 0.51:
        north_wall = 1
    if east_lidar < 0.51:
        east_wall = 1
    if south_lidar < 0.51:
        south_wall = 1
    lidar_wall_detected = {"W": west_wall, "N": north_wall, "E": east_wall, "S":south_wall}
    print(f"Observed Walls: {lidar_wall_detected}")
    return lidar_wall_detected

def output_probs(probs,best_cell):
    # Print prob_vals and most likely cells
    print(f"Probabilities: {probs}")
    print(f"Most likely cell(s): {best_cell}")

def start_navig(probs,best_cell):
    north_lidar = josh.get_lidar_range_image()[400]
    east_lidar = josh.get_lidar_range_image()[600]
    west_lidar = josh.get_lidar_range_image()[200]
    south_lidar = josh.get_lidar_range_image()[0]
    if north_lidar > 0.51:
        josh.straight_move(tot_dist,1)
        #move_a_meter()
        josh.stop()
        fix_orientation()
        #print("test")
        return
    elif west_lidar > 0.51:
        if josh.get_compass_reading() in range(0, 359):
            #print("ye")
            while josh.experiment_supervisor.step(josh.timestep) != -1:
                if josh.get_compass_reading() not in range(179, 181): #turn west
                    josh.rotate_in_place(2)
                    #print("YES")
                else:
                    josh.straight_move(tot_dist,1)
                    josh.stop()
                    print("test2")
                    fix_orientation()
                    return
    #go east
    elif east_lidar > 0.51:
        if josh.get_compass_reading() in range(0, 359):
            #print("yeet")
            while josh.experiment_supervisor.step(josh.timestep) != -1:
                if josh.get_compass_reading() not in range(0,2): #turn east
                    josh.rotate_in_place(2)
                    #print("YES south")
                else:
                    josh.straight_move(tot_dist,1)
                    josh.stop()
                    #print("test2 south")
                    fix_orientation()
                    return
        # josh.straight_move(0,1)
        # josh.stop()
        # print("test2")
        return
    #go south
    else:
    #if south_lidar > 0.51:
        if josh.get_compass_reading() in range(0, 359):
            #print("yeet")
            while josh.experiment_supervisor.step(josh.timestep) != -1:
                if josh.get_compass_reading() not in range(269, 271): #turn south
                    josh.rotate_in_place(2)
                    #print("YES south")
                else:
                    josh.straight_move(tot_dist,1)
                    josh.stop()
                    #print("test2 south")
                    fix_orientation()
                    return
    # # josh.straight_move(0,1)
    # # josh.stop()
    # # print("test2")
    # return
    # if south_lidar < 0.51:
    print("yes")
    josh.stop()

    return
# face north
def fix_orientation():
    while josh.experiment_supervisor.step(josh.timestep) != -1:
        if josh.get_compass_reading() not in range(89, 91): #turn face north
            josh.rotate_in_place(2)
            #print("YESSSSS")
        else:
            josh.stop()
            #print("success")
            return
    return

def new_cell_reached():
    lidar_wall_detected = get_wall_dist()
    
    # Calculate probabilities for all cells
    prob_vals = localize_robot(maze_cells, lidar_wall_detected)
    
    # Find the cell(s) with the highest probability
    max_prob = max(prob_vals.values())
    most_likely_cells = [cell for cell, prob in prob_vals.items() if prob == max_prob]
    output_probs(prob_vals, most_likely_cells)
    #start_navig(prob_vals,most_likely_cells)
    return prob_vals, most_likely_cells
    #josh.stop()

# create function for straight line motion

# Main loop
while josh.experiment_supervisor.step(josh.timestep) != -1:
    tot_dist =0
    # # Get lidar_walls walls
    # lidar_wall_detected = get_wall_dist()
    # # Calculate prob_vals for all cells
    # prob_vals = localize_robot(maze_cells, lidar_wall_detected)
    # # Find the cell(s) with the highest probability
    # max_prob = max(prob_vals.values())
    # most_likely_cells = [cell for cell, prob in prob_vals.items() if prob == max_prob]
    # output_probs(prob_vals, most_likely_cells)
    prob_vals, most_likely_cells = new_cell_reached()
    start_navig(prob_vals,most_likely_cells)
    #break
    # Stop the robot for testing
    #josh.stop()
