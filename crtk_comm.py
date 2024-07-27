import crtk


class crtk_comm:
	def __init__(self, ral):
		self.ral = ral  # ros abstraction layer, what does it do?

		# rostopics that will be used
		self.crtk_utils = crtk.utils(self, self.ral)
		self.crtk_utils.add_operating_state()
		self.crtk_utils.add_servo_cp()
		self.crtk_utils.add_measured_cp()
