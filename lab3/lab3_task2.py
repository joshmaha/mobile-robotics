

# import MyRobot Class and math library
from fairis_tools.my_robot import MyRobot

# create robot instance
josh = MyRobot()

# load environment from maze file
maze_file = '../../worlds/Fall24/maze6.xml'
josh.load_environment(maze_file)

# Move robot to starting position
josh.move_to_start()

# Define constants for the Bug 0 algorithm
wall = "L" # switch wall
rotation_speed = 2.5
flag= True
def wall_following_main():
    
    print("Wall following...")
    l_speed, r_speed = josh.wall_following(0.4, 8,wall) # min_dist, Kp , wall
    
    #print("LEFT SPEED: ", round(l_speed, 2), "RIGHT SPEED: ", round(r_speed, 2))
    josh.set_left_motors_velocity(l_speed)
    josh.set_right_motors_velocity(r_speed)
    flag=True
    #make 45 degree turn when lidar detects object too close to it
    if((fd<= 0.4) and (wall == 'R')):
        josh.turn_in_place(45,"L") #use 45 degree turn for easier turns
    elif((fd <= 0.4 )and (wall == 'L')):
        josh.turn_in_place(-45,"R")

while josh.experiment_supervisor.step(josh.timestep) != -1:
    obstacles = josh.rgb_camera.getRecognitionObjects()
    # if object is detected then call motion to goal method
    #josh.bug0(obstacles)
    vel = josh.pid_straight(1, 15) #default target_d = 1, Kp = 0.1
    # ensure velocity stays within reasonable bounds
    sat_vel = josh.speed_sat(vel)
    #get forward distance from wall
    fd = min(josh.get_lidar_range_image()[350:450])
    right_dist = min(josh.get_lidar_range_image()[401:600])
    left_dist = min(josh.get_lidar_range_image()[200:399])
    min_dist = 0.55
    
    if len(obstacles) == 0: #fd<1.1 and
        if fd<1.5:
            #print("Wall following...")
            print("Rotating robot...")
            #print("")
            #josh.stop()
        if wall == "L":
            josh.rotate_in_place(-rotation_speed)
        else:
            josh.rotate_in_place(rotation_speed)
            #josh.rotate_in_place(speed=2.5)
    else:
        wall_following_main()
    
    # only do motion to goal when not in wall following state
    if len(obstacles) > 0 and (left_dist>1.5 or right_dist>1.5):
        print(" ")
        print("Motion to goal...")
        josh.bug0(obstacles)
        # if x_pos < 0.5:
        # print(f"Final position from object: {x_pos:.2f} metres. Goal reached")
        # josh.stop()
        # break #stop robot and output
