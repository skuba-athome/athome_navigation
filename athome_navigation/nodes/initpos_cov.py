#! /usr/bin/python

import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped

def init_pos():
    nav_init = rospy.Publisher('initialpose', PoseWithCovarianceStamped)
    rospy.init_node('initpos_cov')
    
    while not rospy.is_shutdown():
        initial_pose = rospy.wait_for_message('initialpose2', PoseWithCovarianceStamped)
        initial_pose.pose.covariance = [0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0000001]
        nav_init.publish(initial_pose)

if __name__ == '__main__':
    try:
        init_pos()
    except rospy.ROSInterruptException: pass
