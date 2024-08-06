import math

node = "raven2"
arm_namespaces = ["/arm1", "/arm2"]
gripper_namespaces = ["/grasp1", "/grasp2"]
publish_rate = 1000
max_pos = 0.03 / publish_rate
max_rot = 0.5 / publish_rate
max_grip = (math.pi / 4) / publish_rate
