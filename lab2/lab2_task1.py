# import MyRobot Class and math library
from fairis_tools.my_robot import MyRobot
import math

# create robot instance
josh = MyRobot()

# load environment from maze file
maze_file = '../../worlds/Fall24/maze2.xml'
josh.load_environment(maze_file)

# place robot at (-3, 0, 0)
josh.move_to_start()

# gain constants after multiple tests
Kp = 10 # proportional gain
Ki = 0.05 # integral gain
Kd = 0.1 # derivative gain

# initialize error terms
prev_error = 0 # previous error for derivative
cummulative_error = 0 # integral sum of errors
target_d = 1.0 # 1 meter from front wall
dt = 0.032 # duration of a single timestep

# print robot start pos
start_pos = josh.starting_position
print(f"Starting Position: x={start_pos.x}, y={start_pos.y}, theta={start_pos.theta}.")
print(f"Best gain constants: Proportional gain={Kp}, Integral gain={Ki}, Derivative gain={Kd}.")

while josh.experiment_supervisor.step(josh.timestep) != -1:
 # use front LIDAR reading to get forward distancee
 fd = josh.get_lidar_range_image()[400]

 # if fd is infinity then set it to a measurable distance within the confines of the maze
 if math.isinf(fd):
    fd = 7
 #print("Forward Distance: Out of range (inf), setting to 7 meters")
 else:
    print(f"Forward Distance: {fd}")

 # PID controller
 error = fd - target_d # distance error = (current - desired)
 cummulative_error += error * dt # add error over time for integral
 deriv = (error - prev_error) / dt # rate of error change
 # robot velocity based on PID
 P = Kp * error
 I = Ki * cummulative_error
 D = Kd * deriv
 motor_vel = P + I + D # total motor control saturated velocity signal
 # function for setting motor max velocity
 if motor_vel > 26:
    motor_vel = 26
 elif motor_vel < -26:
    motor_vel = -26
 else:
    motor_vel = motor_vel
 # handle forward and backward distance
 if fd < target_d: # move back
    motor_vel = -abs(motor_vel) # move backward
 else:
    motor_vel = abs(motor_vel) # move forward
 # set motor velocities based on control signal, must be a valid number
 if not math.isnan(motor_vel):
    josh.set_left_motors_velocity(motor_vel)
    josh.set_right_motors_velocity(motor_vel)
 else:
    print("Error: Control signal is NaN, skipping motor update.")
 # store error for next iteration
 prev_error = error
