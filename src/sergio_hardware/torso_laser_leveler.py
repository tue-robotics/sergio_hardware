#!/usr/bin/python

import rospy
from sensor_msgs.msg import JointState

rospy.init_node("torso_laser_leveler")
pub = rospy.Publisher("/sergio/neck/references", JointState, queue_size=1)

r = JointState()
r.name = ["laser_tilt_joint"]
r.position = [0.27]

rospy.sleep(10)

pub.publish(r)
