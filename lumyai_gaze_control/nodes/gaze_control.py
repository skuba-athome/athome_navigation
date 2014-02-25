#! /usr/bin/python
import roslib
roslib.load_manifest('lumyai_gaze_control')
import rospy
import tf
from geometry_msgs.msg import Quaternion, Twist
from math import pi

class GazeControl(object):
	def __init__(self):
		rospy.init_node('gaze_control')

		self.gaze_cmd = rospy.Publisher("/pan_tilt_cmd", Quaternion)
		#rospy.Subscriber("/cmd_vel", Twist, self.vel_callback)
		

		rospy.loginfo("Gaze Control")
		q = tf.transformations.quaternion_from_euler(0, 50.0*pi/180.0, 0)
		quaternion = Quaternion(q[0], q[1], q[2], q[3])
		rate = rospy.Rate(1.0)
		while not rospy.is_shutdown():
			self.gaze_cmd.publish(quaternion)
			rate.sleep()

	#def vel_callback(self,vel):
	#	q = tf.transformations.quaternion_from_euler(0, , self.odomPose.theta)
	#	quaternion = Quaternion(q[0], q[1], q[2], q[3])
	#	self.gaze_cmd.publish(quaternion)
			
if __name__ == '__main__':
	GazeControl()
