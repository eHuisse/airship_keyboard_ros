#This message contain the command of each motor on the airship. The basis of the airship
#is like : Z axis pointing down ; Y axis pointing right of the airship ; X axis pointing front

Header header            	# timestamp in the header is the acquisition time of command

float32 motor_gimbal_angle	# The gimbal of motor is mooving (rad)

float32 left_motor			# Left motor thrust (0->1)
float32 right_motor			# Right motor thrust (0->1)

float32 tail_yaw			# Tail_yaw angle (rad)
float32 tail_pitch			# Tail_pitch angle (rad)