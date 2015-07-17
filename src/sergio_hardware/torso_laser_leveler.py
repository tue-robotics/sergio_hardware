#!/usr/bin/python

import rospy, tf
from sensor_msgs.msg import JointState
import sys
import math

last_update = None
def cb(m):
    if 'laser_tilt_joint' not in m.name:
        return
    
    global last_update
    global listener
    global pub
    
    if last_update is None:
        last_update = rospy.Time.now()

    if (rospy.Time.now() - last_update).to_sec() > 1:
        try:
            pitch = tf.transformations.euler_from_quaternion(listener.lookupTransform("/sergio/base_link","/sergio/torso_laser", rospy.Time(0))[1])[1]
            pitch_torso = tf.transformations.euler_from_quaternion(listener.lookupTransform("/sergio/base_link","/sergio/torso_laser", rospy.Time(0))[1])[1]
        except:
            rospy.logwarn("Could not get tf from base to torso laser")
            return
        last_update = rospy.Time.now()

        r = JointState()

        r.name = ["laser_tilt_joint"]
        ref = pitch - m.position[0]  
        r.position = [ref]

        pub.publish(r)
        rospy.signal_shutdown("done")

rospy.init_node("torso_laser_leveler")
pub = rospy.Publisher("neck/references", JointState, queue_size=1)
sub = rospy.Subscriber("neck/measurements", JointState, cb)
listener = tf.TransformListener()


rospy.spin()
