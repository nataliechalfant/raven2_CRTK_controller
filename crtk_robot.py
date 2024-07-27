import time

import crtk
import crtk_comm


class crtk_robot:
	def __init__(self, node, namespaces):

		self.ral = []
		self.arm = []
		self.target_cp = []
		self.type = node

		# Create rals for listed arms
		for i in range(len(namespaces)):
			self.ral.append(crtk.ral(node, namespaces[i]))

		# Initialize CRTK for listed arms
		for j in range(len(self.ral)):
			self.arm.append(crtk_comm.crtk_comm(self.ral[j]))

		# Check connections and spin
		for k in range(len(self.ral)):
			self.ral[k].check_connections()
			self.ral[k].spin()

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
		for i in range(len(self.arm)):
			self.target_cp.append(self.arm[i].measured_cp())

	def get_type(self):
		return self.type

	def shutdown(self):
		for i in range(len(self.ral)):
			self.ral[i].shutdown()
