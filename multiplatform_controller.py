import time
import PyKDL
import numpy
import xbox_inputs
import crtk_robot
import raven2_defs

# Safety limits
max_pos = raven2_defs.max_pos
max_rot = raven2_defs.max_rot
max_grip = raven2_defs.max_grip

# Controller mode
# 0: two arm mode
# 1+: one arm mode
mode = 1
max_arms = 2

# Timing
start_time = time.time()
last_second = time.time()
loops_per_second = 1


def generate_one_arm_cr(xbc):
	global max_pos, max_rot
	# generate pos_cr
	x_pos = xbc.get_lj_y() * max_pos
	y_pos = xbc.get_lj_x() * max_pos
	z_pos = (xbc.get_lt() - xbc.get_rt()) * max_pos
	pos = PyKDL.Vector(x_pos, y_pos, z_pos)
	# generate rot_cr
	x_rot = xbc.get_rj_x() * -max_rot
	y_rot = xbc.get_rj_y() * max_rot
	z_rot = (xbc.get_lb() - xbc.get_rb()) * max_rot
	rot = PyKDL.Rotation.RPY(x_rot, y_rot, z_rot)

	return PyKDL.Frame(rot, pos)


def generate_two_arm_cr(xbc):
	global max_pos, max_rot
	# generate arm1_cr
	x_pos1 = xbc.get_lj_y() * max_pos
	y_pos1 = xbc.get_lj_x() * max_pos
	if xbc.get_lb():
		lz_mod = 1
	else:
		lz_mod = -1
	z_pos1 = (xbc.get_lt() * lz_mod) * max_pos
	pos1 = PyKDL.Vector(x_pos1, y_pos1, z_pos1)
	# generate arm2_cr
	x_pos2 = xbc.get_rj_y() * max_pos
	y_pos2 = xbc.get_rj_x() * max_pos
	if xbc.get_rb():
		rz_mod = 1
	else:
		rz_mod = -1
	z_pos2 = (xbc.get_rt() * rz_mod) * max_pos
	pos2 = PyKDL.Vector(x_pos2, y_pos2, z_pos2)

	return [PyKDL.Frame(PyKDL.Rotation(), pos1), PyKDL.Frame(PyKDL.Rotation(), pos2)]


def generate_one_arm_gripper(xbc):
	global max_grip
	if xbc.get_a():
		grip = [max_grip]
	elif xbc.get_b():
		grip = [-max_grip]
	else:
		grip = [0]

	return numpy.array(grip, dtype=float)


def generate_two_arm_gripper(xbc):
	global max_grip
	# generate grip1
	if xbc.get_a():
		grip1 = [max_grip, 0, 0]
	elif xbc.get_b():
		grip1 = [-max_grip, 0, 0]
	else:
		grip1 = [0, 0, 0]
	# generate grip2
	if xbc.get_x():
		grip2 = [max_grip, 0, 0]
	elif xbc.get_y():
		grip2 = [-max_grip, 0, 0]
	else:
		grip2 = [0, 0, 0]

	return [numpy.array(grip1, dtype=float), numpy.array(grip2, dtype=float)]


def update_mode(xbc):
	global mode, max_arms
	if xbc.get_back():
		mode = 0
		print("Two arm mode")
	if xbc.get_start():
		if mode:
			if mode < max_arms:
				mode += 1
			else:
				mode = 1
		else:
			mode = 1
		print("One arm mode: arm ", mode)


def check_loop_time():
	global start_time
	time.sleep(max(start_time - time.time(), 0))
	start_time += 0.001


def show_loop_freq():
	global last_second, loops_per_second
	if time.time() - last_second >= 1:
		print("freq: ", loops_per_second)
		loops_per_second = 1
		last_second = time.time()
	else:
		loops_per_second += 1


def main_control(robots, xbc):
	global mode, start_time
	start_time = time.time()

	while True:
		update_mode(xbc)
		# Two arm mode
		if mode == 0:
			pos = generate_two_arm_cr(xbc)
			for robot in robots:
				try:
					for i in range(2):
						robot.update_target_cp(i, pos[i])
						robot.arms[i].servo_cp(robot.get_target_cp(i))
						robot.grippers[i].servo_jr(generate_two_arm_gripper(xbc)[i])
						print(robot.grippers[i].setpoint_js())
				except IndexError:
					print(robot.get_type(), " does not have two arms")
		# One arm mode
		else:
			pos = generate_one_arm_cr(xbc)
			# print(generate_one_arm_gripper(xbc))
			for robot in robots:
				try:
					robot.update_target_cp(mode - 1, pos)
					robot.arms[mode - 1].servo_cp(robot.get_target_cp(mode - 1))
					robot.grippers[mode - 1].servo_jr(generate_one_arm_gripper(xbc))
					# print(robot.grippers[mode - 1].measured_js())

				except IndexError:
					print(robot.get_type(), " does not have the selected arm")

		check_loop_time()
		show_loop_freq()


def main():
	# Initialize xbox controller
	try:
		xbc = xbox_inputs.xbox_inputs()
	except IndexError:
		print("No controller detected. Please connect a XBox controller and relaunch the controller.")
		exit()

	# Initialize physical raven2
	r2p = crtk_robot.crtk_robot(raven2_defs.node, raven2_defs.arm_namespaces, raven2_defs.gripper_namespaces)
	robots = [r2p]

	try:
		main_control(robots, xbc)
	except KeyboardInterrupt:
		for robot in robots:
			robot.shutdown()
		exit()

if __name__ == '__main__':
	main()

# try:
# 	while(True):

		# arm1_curr_m_cp = raven_arm1.measured_cp()
		# arm2_curr_m_cp = raven_arm2.measured_cp()
		# arm1_curr_s_cp = raven_arm1.setpoint_cp()
		# arm2_curr_s_cp = raven_arm2.setpoint_cp()
		#
		# print("arm 1 measured, gold:\n ", arm1_curr_m_cp, "\n")
		# print("arm 2 measured, green:\n ", arm2_curr_m_cp, "\n")
		#
		# print("arm 1 setpoint, gold:\n ", arm1_curr_s_cp, "\n")
		# print("arm 2 setpoint, green:\n ", arm2_curr_s_cp, "\n")

		# max_p = 0.00003
		# arm1_x_p = xbc.get_lj_y() * max_p
		# arm1_y_p = xbc.get_lj_x() * max_p
		# arm1_z_p = (xbc.get_lt() - xbc.get_rt()) * max_p
		#
		# arm1_vec_p = PyKDL.Vector(arm1_x_p, arm1_y_p, arm1_z_p)
		#
		# max_r = 0.0005
		# arm1_x_r = xbc.get_rj_y() * max_r
		# arm1_y_r = xbc.get_rj_x() * max_r
		# arm1_z_r = (xbc.get_lb() - xbc.get_rb()) * max_r
		#
		# arm1_mat_r =PyKDL.Rotation.RPY(arm1_x_r, arm1_y_r, arm1_z_r)
		#
		# arm1_target_cp.p += arm1_vec_p
		# arm1_target_cp.M = arm1_mat_r * arm1_target_cp.M

		# arm2_curr_cp.p += vec_r

		# print(arm1_target_cp)
		# raven_arm1.servo_cp(arm1_target_cp)
		# raven_arm2.servo_cp(arm2_curr_cp)
#
# 		time.sleep(0.001)
# except KeyboardInterrupt:
	# arm1_ral.shutdown()

# print(raven_arm1.measured_cp())
