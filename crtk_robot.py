import time

import crtk
import crtk_comm


class crtk_robot:

	class Arm:
		def __init__(self, ral):
			# Create required rostopics for an arm
			self.crtk = crtk.utils(self, ral)
			self.crtk.add_operating_state()
			self.crtk.add_setpoint_cp()
			self.crtk.add_measured_cp()
			self.crtk.add_servo_cp()

	class Gripper:
		def __init__(self, ral):
			# Create required rostopics for a grasper
			self.crtk = crtk.utils(self, ral)
			self.crtk.add_setpoint_js()
			self.crtk.add_measured_js()
			self.crtk.add_servo_jr()

	def __init__(self, node, arm_namespaces, gripper_namespaces):

		self.ral = crtk.ral(node)
		self.arms = []
		self.grippers = []
		self.target_cp = []
		self.type = self.ral.node_name()

		self.rate = 1000
		self.sleep_rate = self.ral.create_rate(self.rate)

		# Initialize CRTK for listed arms
		for arm in arm_namespaces:
			self.arms.append(self.Arm(self.ral.create_child(arm)))

		# Initialize CRTK for listed grippers
		for gripper in gripper_namespaces:
			self.grippers.append(self.Gripper(self.ral.create_child(gripper)))

		# Check connections and spin
		self.ral.check_connections()
		self.ral.spin()

		# Set initial target Cartesian Points
		self.reset_target_cp()

	def get_target_cp(self, arm):
		return self.target_cp[arm]

	def set_target_cp(self, arm, target_cp):
		self.target_cp[arm] = target_cp

	def update_target_cp(self, arm, cr):
		self.target_cp[arm].p += cr.p
		self.target_cp[arm].M = cr.M * self.target_cp[arm].M

	def reset_target_cp(self):
		self.target_cp = []
		for i in range(len(self.arms)):
			self.target_cp.append(self.arms[i].measured_cp())

	def get_type(self):
		return self.type

	def shutdown(self):
		for i in range(len(self.ral)):
			self.ral[i].shutdown()
