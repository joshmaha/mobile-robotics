

from fairis_lib.robot_lib.rosbot import RosBot
import math, time

class MyRobot(RosBot):
    def __init__(self):
        RosBot.__init__(self)
        self.d_mid = self.axel_length / 2 # 0.1325 meters
        self.ang_vel = 7.5 # rad/sec
        self.lin_vel = self.ang_vel * self.wheel_radius # in m/s
        self.x = 0
        self.y = 0
        self.theta = 0.0
        self.prev_left_distance = 0.0
        self.prev_right_distance = 0.0
        self.prev_distance = 0
    
    # def rotate_inplace(self,angle,distance):
    # # set motor velocities
    # self.set_left_motors_velocity(2)
    # self.set_right_motors_velocity()
    #straight line movements
    def straight_move(self, dist, add_dist):
        self.set_right_motors_velocity(self.ang_vel)
        self.set_left_motors_velocity(self.ang_vel)
        dist += add_dist
        straight_time = add_dist / self.lin_vel
        #self.total_time += straight_time
        # Print distance and velocity
        #print(f"Left wheel velocity= {round(self.lin_vel, 1)} m/s, Right wheel velocity= {round(self.lin_vel, 1)} m/s, Distance= {round(add_dist, 1)} m, Time= {round(straight_time, 1)} secs")
        #
        while self.experiment_supervisor.step(self.timestep) != -1:
            dist_front_L_wheel = (self.wheel_radius * self.get_front_left_motor_encoder_reading())
            dist_front_R_wheel = (self.wheel_radius * self.get_front_right_motor_encoder_reading())
            #change between lab 4 tasks 1 and 2
            #self.pose_update()
            if dist_front_L_wheel > dist or dist_front_R_wheel > dist:
                return dist
    #circular movements for ONLY 0.5m radius circles given inner wheel
    def arc_move(self, dist, in_wheel):
        arc_radius = 0.5
        in_wheel_dist = (2 * math.pi * (arc_radius - self.d_mid)) / 4
        out_wheel_dist = (2 * math.pi * (arc_radius + self.d_mid)) / 4
        ang_vel_curve = self.lin_vel / (arc_radius + self.d_mid)
        lin_vel_in_wheel = ang_vel_curve * (arc_radius - self.d_mid)
        in_wheel_ang_vel = lin_vel_in_wheel / self.wheel_radius
        if in_wheel == "L":
            self.set_right_motors_velocity(self.ang_vel)
            self.set_left_motors_velocity(in_wheel_ang_vel)
            dist += in_wheel_dist
        elif in_wheel == "R":
            self.set_right_motors_velocity(in_wheel_ang_vel)
            self.set_left_motors_velocity(self.ang_vel)
            dist += out_wheel_dist
        else:
            self.stop()
        curve_dist = (2 * math.pi * arc_radius) / 4
        curve_lin_vel = (lin_vel_in_wheel + self.lin_vel) / 2
        curve_time = curve_dist / curve_lin_vel
        self.total_time += curve_time
        #self.pt_L_velocities.append(self.lin_vel),
        self.pt_R_velocities.append(lin_vel_in_wheel), self.pt_time.append(self.total_time)
        while self.experiment_supervisor.step(self.timestep) != -1:
            dist_front_L_wheel = (self.wheel_radius *
                                  self.get_front_left_motor_encoder_reading())
            dist_front_R_wheel = (self.wheel_radius *
                                  self.get_front_right_motor_encoder_reading())
            self.pose_update()
            if dist_front_L_wheel > dist:
                return dist
    # distance robot wheels travel
    def calculate_wheel_distance_traveled(self, previous_encoder_reading):
    # Get the current encoder readings as a list: [front_left, front_right, rear_left, rear_right]
        curr_encoder_readings = self.get_encoder_readings()
        #avg current left and right sides
        curr_left_avg = (curr_encoder_readings[0] + curr_encoder_readings[2]) / 2
        # Average of front_left and rear_left
        curr_right_avg = (curr_encoder_readings[1] + curr_encoder_readings[3]) / 2
        # Average of front_right and rear_right
        # avg previous encoder readings
        prev_left_avg = (previous_encoder_reading[0] + previous_encoder_reading[2])/ 2 # Average of previous front_left and rear_left
        prev_right_avg = (previous_encoder_reading[1] +
        previous_encoder_reading[3]) / 2 # Average of previous front_right and rear_right
        #calculate distance traveled
        delta_left = (curr_left_avg - prev_left_avg) * self.wheel_radius # Convert to linear distance
        delta_right = (curr_right_avg - prev_right_avg) * self.wheel_radius # Convert to linear distance
        #return avg wheel travel distance
        return ((delta_left + delta_right)/2)
    
    #takes starting x and y cordinates
    def pose_update(self):
        # get encoder readings (in meters)
        curr_front_left_d = self.get_front_left_motor_encoder_reading() * self.wheel_radius
        curr_rear_left_d = self.get_rear_left_motor_encoder_reading() * self.wheel_radius
        curr_front_right_d = self.get_front_right_motor_encoder_reading() * self.wheel_radius
        curr_rear_right_d = self.get_rear_right_motor_encoder_reading() * self.wheel_radius
        # average how much left and right wheels have moved since prior update
        delta_left = ((curr_front_left_d + curr_rear_left_d) / 2) - self.prev_left_distance
        delta_right = ((curr_front_right_d + curr_rear_right_d) / 2) - self.prev_right_distance
        # average the wheel movements to estimate the total distance traveled
        delta_d = (delta_left + delta_right) / 2
        # get the current compass reading in degrees and convert to radians
        compass_degrees = self.get_compass_reading()
        self.theta = math.radians(compass_degrees) # Update heading angle
        # find change in x and y based on the movement
        delta_x = delta_d * math.cos(self.theta)
        delta_y = delta_d * math.sin(self.theta)
        # update x and y positions
        self.x += delta_x
        self.y += delta_y
        # update previous encoder values
        self.prev_left_distance = ((curr_front_left_d + curr_rear_left_d) / 2)
        self.prev_right_distance = ((curr_front_right_d + curr_rear_right_d) / 2)
        # output updated position
        print(f"Current Pose: x = {self.x:.2f}, y = {self.y:.2f}, theta = {math.degrees(self.theta):.2f}°")
        
    #return position values of current x, y and theta
    def get_pose(self):
        return self.x, self.y, math.degrees(self.theta)
    
    # rotate robot by given angle
    def turn_in_place(self, angle, direction):
        # Convert angle to radians for calculations
        angle_rad = math.radians(angle)
        # Calculate the arc length the wheels need to travel
        # Arc length = (Axle length / 2) * angle (in radians)
        arc_length = self.d_mid * angle_rad
        # Calculate how long the wheels should move to complete the turn
        # Time required = arc_length / (wheel_radius * angular velocity)
        # Assume constant angular velocity for both wheels.
        angular_velocity = 2.0 # Adjust as needed (radians/second)
        time_to_turn = arc_length / (self.wheel_radius * angular_velocity)
        # Set wheel velocities for in-place turning
        if direction == 'L':
            left_velocity = -angular_velocity # Left wheel moves backward
            right_velocity = angular_velocity # Right wheel moves forward
        elif direction == 'R':
            left_velocity = angular_velocity # Left wheel moves forward
            right_velocity = -angular_velocity # Right wheel moves backward
        else: #error handling
            print("Invalid direction! Use 'l' for left and 'r' for right.")
            return
        # set motor velocities
        self.set_left_motors_velocity(left_velocity)
        self.set_right_motors_velocity(right_velocity)
        # keep motors running for calculated time
        start_time = time.time()
        while (time.time() - start_time) < time_to_turn:
            # continue to turn until time has passed
            pass
    
    #saturation function
    def speed_sat(self,vel):
        max_v = 20
        if vel >= max_v :
            vel = max_v
        elif vel <= -max_v:
            vel = -max_v
        return vel
    
    #pid function
    def pid_straight(self, target_d=1, Kp=0.1):
        fd = min(self.get_lidar_range_image()[350:450]) # use front LIDAR reading to get forward distance
        error = fd - target_d # front distance error = current - desired
        return self.speed_sat(Kp * error) # return saturated velocity value so that robot does not move recklessly
    #pid function for only wall following strictly
    
    def wall_following(self, min_dist, Kp, wall):
        motor_vel = self.pid_straight(0.5, 10)
        right_d = min(self.get_lidar_range_image()[401:600])  # north most right lidar to south-east point range
        left_d = min(self.get_lidar_range_image()[200:399])   # north most left lidar to south-west point range
        if wall == 'L':
            error = min_dist - right_d  # right distance error = current - desired
            if (error > 0):  # too close, slow down left wheel
                right_vel = self.speed_sat(motor_vel)
                left_vel = self.speed_sat(motor_vel - abs(Kp * error))
            elif (error < 0):  # too far, slow down right wheel
                left_vel = self.speed_sat(motor_vel)
                right_vel = self.speed_sat(motor_vel - abs(Kp * error))
            elif left_d <= 0.55:  # too close to left wall, quickly stop left and move right
                error = 0.55 - left_d
                right_vel = self.speed_sat(motor_vel - abs(Kp * error))
                left_vel = self.speed_sat(0)
            else:  # distance is fine, move wheels at same speed
                left_vel = motor_vel
                right_vel = motor_vel
        else:  # left wall scenario same movements for opposite conditions
            error = min_dist - left_d
            if (error > 0):
                left_vel = self.speed_sat(motor_vel)
                right_vel = self.speed_sat(motor_vel - abs(Kp * error))
            elif (error < 0):
                right_vel = self.speed_sat(motor_vel)
                left_vel = self.speed_sat(motor_vel - abs(Kp * error))
            elif (right_d <= 0.55):
                error = 0.55 - right_d
                right_vel = self.speed_sat(0)
                left_vel = self.speed_sat(motor_vel - abs(Kp * error))
            else:
                left_vel = motor_vel
                right_vel = motor_vel
        return left_vel, right_vel
    
    # rotate robot without angle parameter needed
    def rotate_in_place(self, speed):
        self.set_left_motors_velocity(-speed)
        self.set_right_motors_velocity(speed)
        #motion to goal function for moving towards an obstacle detected
        
    def motion_to_goal(self, rec_objects):
        # determine robot forward velocity using PID
        vel = self.pid_straight(1, 8)
        # ensure velocity stays within reasonable bounds
        sat_vel = self.speed_sat(vel)/2
        # taken from lab3 pdf
        # if object is detected
        if len(rec_objects) > 0:
            # store info of obstacle
            landmark = rec_objects[0]
            x_pos = landmark.getPosition()[0]
            y_pos = landmark.getPosition()[1]
            z_pos = landmark.getPosition()[2]
            # output object position relative to the camera [X, Y, Z]
            print(f"Current Pose from object: x = {x_pos:.2f}, y = {y_pos:.2f}, z = {z_pos:.2f}")
            # check that robot is centered in camera's view by rotating until within 0.1 metres of center view
            if 0.1 > y_pos > -0.1:
                # move forward
                print("Object identified. Motion to goal in progress")
                print(f"Speed: {(sat_vel):.2f} m/s")
                # use saturation and pid for smooth and controlled movement
                self.set_left_motors_velocity(sat_vel)
                self.set_right_motors_velocity(sat_vel)
                # stop robot if obstacle is 0.5 metres away using x cordinate
                if landmark.getPosition()[0] < 0.5:
                    print(f"Final position from object: {x_pos:.2f} metres. Goal reached")
                    self.stop()
                    return #stop robot and output
    
    def get_close_to_wall(self,fd):
        if fd<1.1: #first wall encountered this works
            print("Obstacle detected, turning robot...")
            print(f"fd:{fd}")
            self.stop()
    
    #motion to goal function for moving towards an obstacle detected
    def bug0(self, rec_objects):
        # determine robot forward velocity using PID
        vel = self.pid_straight(0.01, 20) #default target_d = 1, Kp = 0.1
        # ensure velocity stays within reasonable bounds
        sat_vel = self.speed_sat(vel)/2
        if len(rec_objects) > 0:
            # store info of obstacle
            landmark = rec_objects[0]
            x_pos = landmark.getPosition()[0]
            y_pos = landmark.getPosition()[1]
            z_pos = landmark.getPosition()[2]
            # output object position relative to the camera [X, Y, Z]
            # check that robot is centered in camera's view by rotating until within 0.1 metres of center view
            if not (0.5 > y_pos > -0.5):
                self.rotate_in_place(speed=2.5)
            # check that robot is centered in camera's view by rotating until within 0.1 metres of center view
            if 0.5 > y_pos > -0.5:
                # move forward
                print("Object identified. Motion to goal in progress")
                print(f"Speed: {(sat_vel):.2f} m/s")
                # use saturation and pid for smooth and controlled movement
                self.set_left_motors_velocity(sat_vel)
                self.set_right_motors_velocity(sat_vel)
                # stop robot if obstacle is 0.5 metres away using x cordinate
                if landmark.getPosition()[0] < 0.5:
                    print(f"Final position from object: {x_pos:.2f} metres. Goal reached")
                    self.stop()
                    return #stop robot and output
    def check_for_landmarks(self):
        rec_objects = self.rgb_camera.getRecognitionObjects()
        print(" ") # Space between outputs to make it more readable
        self.turn_in_place(self, 360, "L")
        if len(rec_objects) > 0:
            # store info of obstacle
            l1 = rec_objects[0]
            x_pos = l1.getPosition()[0]
            y_pos = l1.getPosition()[1]
            z_pos = l1.getPosition()[2]
