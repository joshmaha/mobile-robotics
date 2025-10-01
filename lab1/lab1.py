# import libraries to use
import math
from matplotlib import pyplot as plt
# import MyRobot Class
from fairis_tools.my_robot import MyRobot
# create the robot instance.
robot = MyRobot()
# load environment from maze file
maze_file = '../../worlds/Fall24/maze1.xml'
robot.load_environment(maze_file)
# physical robot specs
d_mid = robot.axel_length/2 #0.1325 meters
ang_vel = 7.5 # rad/sec
lin_vel = ang_vel * robot.wheel_radius # in m/s
# place robot at (2, -2, math.pi)
robot.move_to_start()
tot_dist = 0 # will use variable for final robot distance
# initial position (x, y, theta)
x = 2.0
y = -2.0
import math
from matplotlib import pyplot as plt

# import MyRobot Class
from fairis_tools.my_robot import MyRobot


# create the robot instance and environment
robot = MyRobot()
maze_file = '../../worlds/Fall24/maze1.xml'
robot.load_environment(maze_file)
robot.move_to_start()

# physical robot specs
d_mid = robot.axel_length / 2  # 0.1325 meters
ang_vel = 7.5  # rad/sec (wheel angular velocity used for motor commands)
lin_vel = ang_vel * robot.wheel_radius  # nominal linear velocity (m/s)

# initial pose (estimate)
x = 2.0
y = -2.0
theta = math.pi
prev_left_distance = 0.0
prev_right_distance = 0.0

# data for plotting
x_gps_pts = []
y_gps_pts = []
x_enc_pts = [x]
y_enc_pts = [y]

pt_L_velocities = [0]
pt_R_velocities = [0]
pt_time = [0]
total_time = 0.0


def calc_straight_d(x1, y1, x2, y2):
	"""Euclidean distance between two points."""
	return math.hypot(x2 - x1, y2 - y1)


def calc_curve_d(radius, angle_rad):
	"""Arc length for given radius and angle (angle in radians)."""
	return abs(radius * angle_rad)


def calc_wheels_turn_d(rad_angle):
	"""Equivalent wheel travel distance for a rotation about the robot center."""
	return abs(rad_angle) * d_mid


def calc_left_vel(d, angular_velocity):
	return d / angular_velocity if angular_velocity != 0 else 0


def calc_right_vel(d, angular_velocity):
	return d / angular_velocity if angular_velocity != 0 else 0


def calc_time(d, velocity):
	return d / velocity if velocity != 0 else float('inf')


def print_kinematics():
	"""Compute and print planned segment kinematics (distances/times/approx velocities)."""
	circle_radius = 0.5

	# P0->P1
	d_1 = calc_straight_d(2.0, -2.0, -1.5, -2.0)
	v_L_1 = calc_left_vel(d_1, ang_vel)
	v_R_1 = v_L_1
	time_1 = calc_time(d_1, lin_vel)
	print(f"P0->P1 dist= {d_1:.2f}m, Vl={v_L_1:.2f}m/s, Vr={v_R_1:.2f}m/s, Time={time_1:.2f}secs")

	# P1->P2 (quarter circle)
	d_2 = calc_curve_d(circle_radius, math.pi / 2)
	v_L_2 = calc_left_vel(circle_radius + d_mid, ang_vel)
	v_R_2 = calc_right_vel(circle_radius - d_mid, ang_vel)
	time_2 = calc_time(d_2, lin_vel)
	print(f"P1->P2 dist= {d_2:.2f}m, Vl={v_L_2:.2f}m/s, Vr={v_R_2:.2f}m/s, Time={time_2:.2f}secs")

	# P2->P3 (quarter circle)
	d_3 = calc_curve_d(circle_radius, math.pi / 2)
	v_L_3 = calc_left_vel(circle_radius + d_mid, ang_vel)
	v_R_3 = calc_right_vel(circle_radius - d_mid, ang_vel)
	time_3 = calc_time(d_3, lin_vel)
	print(f"P2->P3 dist= {d_3:.2f}m, Vl={v_L_3:.2f}m/s, Vr={v_R_3:.2f}m/s, Time={time_3:.2f}secs")

	# P3->P4 rotate then straight
	d_rotate_4 = calc_wheels_turn_d(2 * math.pi + (math.pi / 45))
	time_rotate_4 = d_rotate_4 / ang_vel
	straight_d_4 = calc_straight_d(-1.5, -1.0, 1.5, -1.0)
	d_4 = straight_d_4 + d_rotate_4
	v_L_4 = calc_left_vel(d_4, ang_vel) + (d_rotate_4 / time_rotate_4)
	v_R_4 = v_L_4
	time_4 = calc_time(d_4, lin_vel) + time_rotate_4
	print(f"P3->P4 dist= {d_4:.2f}m, Vl={v_L_4:.2f}m/s, Vr={v_R_4:.2f}m/s, Time={time_4:.2f}secs")

	# P4->P5
	d_rotate_5 = calc_wheels_turn_d(2 * math.pi + (-math.pi / 4))
	time_rotate_5 = d_rotate_5 / ang_vel
	straight_d_5 = calc_straight_d(1.5, -1.0, 2.0, -0.5)
	d_5 = straight_d_5 + d_rotate_5
	v_L_5 = calc_left_vel(d_5, ang_vel) + (d_rotate_5 / time_rotate_5)
	v_R_5 = v_L_5
	time_5 = calc_time(d_5, lin_vel) + time_rotate_5
	print(f"P4->P5 dist= {d_5:.2f}m, Vl={v_L_5:.2f}m/s, Vr={v_R_5:.2f}m/s, Time={time_5:.2f}secs")

	# P5->P6
	d_rotate_6 = calc_wheels_turn_d(2 * math.pi + (-math.pi / 2))
	time_rotate_6 = d_rotate_6 / ang_vel
	straight_d_6 = calc_straight_d(2.0, -0.5, 1.5, 0.0)
	d_6 = straight_d_6 + d_rotate_6
	v_L_6 = calc_left_vel(d_6, ang_vel) + (d_rotate_6 / time_rotate_6)
	v_R_6 = v_L_6
	time_6 = calc_time(d_6, lin_vel) + time_rotate_6
	print(f"P5->P6 dist= {d_6:.2f}m, Vl={v_L_6:.2f}m/s, Vr={v_R_6:.2f}m/s, Time={time_6:.2f}secs")

	# P6->P7
	d_rotate_7 = calc_wheels_turn_d(2 * math.pi + (-math.pi / 4))
	time_rotate_7 = d_rotate_7 / ang_vel
	straight_d_7 = calc_straight_d(1.5, 0.0, 0.0, 0.0)
	d_7 = straight_d_7 + d_rotate_7
	v_L_7 = calc_left_vel(d_7, ang_vel) + (d_rotate_7 / time_rotate_7)
	v_R_7 = v_L_7
	time_7 = calc_time(d_7, lin_vel) + time_rotate_7
	print(f"P6->P7 dist= {d_7:.2f}m, Vl={v_L_7:.2f}m/s, Vr={v_R_7:.2f}m/s, Time={time_7:.2f}secs")

	# P7->P8
	d_8 = calc_straight_d(0.0, 0.0, -2.0, 0.0)
	v_L_8 = calc_left_vel(d_8, ang_vel)
	v_R_8 = v_L_8
	time_8 = calc_time(d_8, lin_vel)
	print(f"P7->P8 dist= {d_8:.2f}m, Vl={v_L_8:.2f}m/s, Vr={v_R_8:.2f}m/s, Time={time_8:.2f}secs")

	# P9->P10
	d_rotate_9 = calc_wheels_turn_d(math.pi / 2 + math.pi / 60)
	time_rotate_9 = d_rotate_9 / ang_vel
	straight_d_9 = calc_straight_d(-2.0, 2.0, 1.5, 2.0)
	d_9 = straight_d_9 + d_rotate_9
	v_L_9 = calc_left_vel(d_9, ang_vel) + (d_rotate_9 / time_rotate_9)
	v_R_9 = v_L_9
	time_9 = calc_time(d_9, lin_vel) + time_rotate_9
	print(f"P9->P10 dist= {d_9:.2f}m, Vl={v_L_9:.2f}m/s, Vr={v_R_9:.2f}m/s, Time={time_9:.2f}secs")

	# P10->P11 and P11->P12 (quarter circles)
	d_10 = calc_curve_d(circle_radius, math.pi / 2)
	v_L_10 = calc_left_vel(circle_radius + d_mid, ang_vel)
	v_R_10 = calc_right_vel(circle_radius - d_mid, ang_vel)
	time_10 = calc_time(d_10, lin_vel)
	print(f"P10->P11 dist= {d_10:.2f}m, Vl={v_L_10:.2f}m/s, Vr={v_R_10:.2f}m/s, Time={time_10:.2f}secs")

	d_11 = calc_curve_d(circle_radius, math.pi / 2)
	v_L_11 = calc_left_vel(circle_radius + d_mid, ang_vel)
	v_R_11 = calc_right_vel(circle_radius - d_mid, ang_vel)
	time_11 = calc_time(d_11, lin_vel)
	print(f"P11->P12 dist= {d_11:.2f}m, Vl={v_L_11:.2f}m/s, Vr={v_R_11:.2f}m/s, Time={time_11:.2f}secs")

	# P12->P13
	d_rotate_12 = calc_wheels_turn_d(2 * math.pi + math.pi / 30)
	time_rotate_12 = d_rotate_12 / ang_vel
	straight_d_12 = calc_straight_d(1.5, 1.0, -1.0, 1.0)
	d_12 = straight_d_12 + d_rotate_12
	v_L_12 = calc_left_vel(d_12, ang_vel) + (d_rotate_12 / time_rotate_12)
	v_R_12 = v_L_12
	time_12 = calc_time(d_12, lin_vel) + time_rotate_12
	print(f"P12->P13 dist= {d_12:.2f}m, Vl={v_L_12:.2f}m/s, Vr={v_R_12:.2f}m/s, Time={time_12:.2f}secs")

	total_t = time_1 + time_2 + time_3 + time_4 + time_5 + time_6 + time_7 + time_8 + time_9 + time_10 + time_11 + time_12
	total_d = d_1 + d_2 + d_3 + d_4 + d_5 + d_6 + d_7 + d_8 + d_9 + d_10 + d_11 + d_12
	print(f"Total time:{total_t:.2f}seconds, total distance:{total_d:.2f}metres, total velocity:{(total_d/total_t):.2f}m/s")


def pose_update():
	"""Update estimated pose using encoder and compass readings and sample GPS for plotting."""
	global x, y, theta, prev_left_distance, prev_right_distance

	curr_front_left_d = robot.get_front_left_motor_encoder_reading() * robot.wheel_radius
	curr_rear_left_d = robot.get_rear_left_motor_encoder_reading() * robot.wheel_radius
	curr_front_right_d = robot.get_front_right_motor_encoder_reading() * robot.wheel_radius
	curr_rear_right_d = robot.get_rear_right_motor_encoder_reading() * robot.wheel_radius

	delta_left = ((curr_front_left_d + curr_rear_left_d) / 2) - prev_left_distance
	delta_right = ((curr_front_right_d + curr_rear_right_d) / 2) - prev_right_distance
	delta_d = (delta_left + delta_right) / 2

	compass_degrees = robot.get_compass_reading()
	theta = math.radians(compass_degrees)

	delta_x = delta_d * math.cos(theta)
	delta_y = delta_d * math.sin(theta)

	# update globals
	x += delta_x
	y += delta_y
	prev_left_distance = ((curr_front_left_d + curr_rear_left_d) / 2)
	prev_right_distance = ((curr_front_right_d + curr_rear_right_d) / 2)

	print(f"Current Pose: x = {x:.2f}, y = {y:.2f}, theta = {math.degrees(theta):.2f}°")

	# sample GPS and encoder points for plotting
	gps_values = robot.gps.getValues()
	x_gps_pts.append(gps_values[0])
	y_gps_pts.append(gps_values[1])
	x_enc_pts.append(x)
	y_enc_pts.append(y)


def straight_move(dist, add_dist):
	"""Drive straight for an additional add_dist meters. Returns updated dist target."""
	global pt_L_velocities, pt_R_velocities, pt_time, total_time

	robot.set_right_motors_velocity(ang_vel)
	robot.set_left_motors_velocity(ang_vel)

	target = dist + add_dist
	straight_time = add_dist / lin_vel if lin_vel != 0 else 0
	total_time += straight_time

	print(f"Left wheel velocity= {round(lin_vel, 1)} m/s, Right wheel velocity= {round(lin_vel, 1)} m/s, Distance= {round(add_dist, 1)} m, Time= {round(straight_time, 1)} secs")
	pt_L_velocities.append(lin_vel)
	pt_R_velocities.append(lin_vel)
	pt_time.append(total_time)

	# run until the wheel distances reach the target
	while robot.experiment_supervisor.step(robot.timestep) != -1:
		dist_front_L_wheel = robot.wheel_radius * robot.get_front_left_motor_encoder_reading()
		dist_front_R_wheel = robot.wheel_radius * robot.get_front_right_motor_encoder_reading()
		pose_update()
		if dist_front_L_wheel > target or dist_front_R_wheel > target:
			return target
	return target


def turn_wheels(dist, rad_angle):
	"""Rotate in place by turning wheels in opposite directions. Returns updated dist target."""
	global pt_L_velocities, pt_R_velocities, pt_time

	rot_dist = abs(rad_angle * d_mid)
	if rad_angle < 0:
		# rotate counter-clockwise
		robot.set_left_motors_velocity(-(ang_vel / 4))
		robot.set_right_motors_velocity(ang_vel / 4)
		target = dist - rot_dist
	else:
		# rotate clockwise
		robot.set_left_motors_velocity(ang_vel / 4)
		robot.set_right_motors_velocity(-(ang_vel / 4))
		target = dist + rot_dist

	while robot.experiment_supervisor.step(robot.timestep) != -1:
		dist_front_L_wheel = robot.wheel_radius * robot.get_front_left_motor_encoder_reading()
		pose_update()
		if rad_angle < 0 and dist_front_L_wheel < target:
			return target
		if rad_angle >= 0 and dist_front_L_wheel > target:
			return target
	return target


def arc_move(dist, in_wheel):
	"""Perform a quarter-arc turn where in_wheel is 'L' or 'R' (inner wheel). Returns updated dist target."""
	global pt_L_velocities, pt_R_velocities, pt_time, total_time

	arc_radius = 0.5
	in_wheel_dist = (2 * math.pi * (arc_radius - d_mid)) / 4
	out_wheel_dist = (2 * math.pi * (arc_radius + d_mid)) / 4
	ang_vel_curve = lin_vel / (arc_radius + d_mid)
	lin_vel_in_wheel = ang_vel_curve * (arc_radius - d_mid)
	in_wheel_ang_vel = lin_vel_in_wheel / robot.wheel_radius

	if in_wheel == "L":
		robot.set_right_motors_velocity(ang_vel)
		robot.set_left_motors_velocity(in_wheel_ang_vel)
		target = dist + in_wheel_dist
		print(f"Left wheel velocity= {round(lin_vel_in_wheel, 1)} m/s. Right wheel velocity= {round(lin_vel, 1)} m/s")
	elif in_wheel == "R":
		robot.set_right_motors_velocity(in_wheel_ang_vel)
		robot.set_left_motors_velocity(ang_vel)
		target = dist + out_wheel_dist
		print(f"Left wheel velocity= {round(lin_vel, 1)} m/s. Right wheel velocity= {round(lin_vel_in_wheel, 1)} m/s")
	else:
		robot.stop()
		return dist

	curve_dist = (2 * math.pi * arc_radius) / 4
	curve_lin_vel = (lin_vel_in_wheel + lin_vel) / 2
	curve_time = curve_dist / curve_lin_vel if curve_lin_vel != 0 else 0
	total_time += curve_time
	print(f"Distance= {round(curve_dist, 1)} m, Time= {round(curve_time, 1)} secs")

	pt_L_velocities.append(lin_vel)
	pt_R_velocities.append(lin_vel_in_wheel)
	pt_time.append(total_time)

	while robot.experiment_supervisor.step(robot.timestep) != -1:
		dist_front_L_wheel = robot.wheel_radius * robot.get_front_left_motor_encoder_reading()
		pose_update()
		if dist_front_L_wheel > target:
			return target
	return target


def graph_plotter():
	plt.figure(1)
	plt.plot(x_gps_pts, y_gps_pts, label='GPS points', marker='o', linestyle='dotted', color='b')
	plt.plot(x_enc_pts, y_enc_pts, label='Encoder points', marker='*', linestyle='dashed', color='r')
	plt.title("GPS vs Encoder points")
	plt.xlabel("X(m)")
	plt.ylabel("Y(m)")
	plt.legend()
	plt.grid(True)

	plt.figure(2)
	plt.plot(pt_time, pt_L_velocities, label='Left wheel velocity', marker='o', linestyle='dotted', color='b')
	plt.plot(pt_time, pt_R_velocities, label='Right wheel velocity', marker='*', linestyle='dashed', color='b')
	plt.title("Velocity vs Time")
	plt.xlabel("Time(s)")
	plt.ylabel("Velocity(m/s)")
	plt.legend()
	plt.grid(True)
	plt.show()


if __name__ == '__main__':
	tot_dist = 0
	print_kinematics()

	print("P0->P1:")
	tot_dist = straight_move(tot_dist, 3.5)

	print("P1->P2:")
	tot_dist = arc_move(tot_dist, "R")

	print("P2->P3:")
	tot_dist = arc_move(tot_dist, "R")
	tot_dist = turn_wheels(tot_dist, (2 * math.pi + (math.pi / 45)))

	print("P3->P4:")
	tot_dist = straight_move(tot_dist, 3)

	print("P4->P5:")
	tot_dist = turn_wheels(tot_dist, -math.pi / 4)
	tot_dist = straight_move(tot_dist, 0.707)

	print("P5->P6:")
	tot_dist = turn_wheels(tot_dist, -math.pi / 2)
	tot_dist = straight_move(tot_dist, 0.707)

	print("P6->P7:")
	tot_dist = turn_wheels(tot_dist, -math.pi / 4)
	tot_dist = straight_move(tot_dist, 1.5)

	print("P7->P8:")
	tot_dist = straight_move(tot_dist, 2)
	tot_dist = turn_wheels(tot_dist, math.pi / 2 + math.pi / 60)

	print("P8->P9:")
	tot_dist = straight_move(tot_dist, 2)
	tot_dist = turn_wheels(tot_dist, math.pi / 2 + math.pi / 60)

	print("P9->P10:")
	tot_dist = straight_move(tot_dist, 3.5)

	print("P10->P11:")
	tot_dist = arc_move(tot_dist, "R")

	print("P11->P12:")
	tot_dist = arc_move(tot_dist, "R")
	tot_dist = turn_wheels(tot_dist, (2 * math.pi + (math.pi / 30)))

	print("P12->P13:")
	tot_dist = straight_move(tot_dist, 2.5)

	print(f"Total time:{round(total_time,2)} secs, Total Distance: {round(tot_dist,2)} m")
	robot.stop()
	graph_plotter()
