# import MyRobot Class and math library
from fairis_tools.my_robot import MyRobot
import math,time

# create robot instance
josh = MyRobot()

# load environment from maze file
maze_file = '../../worlds/Fall24/maze4.xml' # switch between maze3 and maze4 for video
josh.load_environment(maze_file)

# Move robot to starting position
josh.move_to_start() # position(-1,1)

#start and end variables to get robot time for completing the maze
t_start = time.time()
t_end = 0
goal_pos = (-1,1) #end goal position

# Main Control Loop for Robot
while josh.experiment_supervisor.step(josh.timestep) != -1:
 wall = 'L' # change between left and right for video
 left_vel,right_vel = josh.wall_following(0.4,25,wall) #min_dist = 0.4m and
 Kp=25
 #print josh position, velocity for both wheels and lidar(front,left and right) readings at all times
 josh.pose_update()
 right_dist = min(josh.get_lidar_range_image()[401:600])
 front_dist = min(josh.get_lidar_range_image()[350:450])
 left_dist = min(josh.get_lidar_range_image()[200:399])
 print(f"Left wheel:{left_vel:.4f} m/s")
 print(f"Right Wheel:{right_vel:.4f} m/s")
 print(f"Right distance:{right_dist:.4f} meters")
 print(f"Front distance:{right_dist:.4f} meters")
 print(f"Left distance:{right_dist:.4f} meters")
 print("") #breakline to appear clearer in console
 josh.set_left_motors_velocity(left_vel)
 josh.set_right_motors_velocity(right_vel)
 # make 45 degree turn when lidar detects object too close to it
 if((front_dist <= 0.4) and (wall == 'R')):
    josh.turn_in_place(45,"L") #use 45 degree turn for easier turns
 elif((front_dist <= 0.4 )and (wall == 'L')):
    josh.turn_in_place(-45,"R")
 x, y, theta = josh.get_pose() # store pose in these variables to print at end
 if abs(goal_pos[0]-x) < 0.3 and abs(goal_pos[1]-y) < 0.3: #testing maze3 for <0.2 x=-0.80 y=1.08 for <0.3 x=-0.7, y=1.03
    print(f"Close enough to the goal! x: {x} y:{y}")
 t_end = time.time() # Stop time after reaching the goal
 # total time
 tot_time = t_end - t_start
 print(f"Total travel time: {tot_time:.4f} seconds")
 josh.stop() # robot stops after reaching within +- 0.5m of goal position (-1,1)
 break

#maze 3 right wall following x=-0.80 y=1.08 time:89 secs
#maze 3 left wall following x=-0.70 y=0.816 time:130 secs
#maze 4 right wall following x=- y= NEVER ENDS
#maze 4 left wall following x=-0.70 y=0.88