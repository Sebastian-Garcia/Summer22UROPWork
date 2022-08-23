#!/usr/bin/env python3

import rospy
import numpy as np
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import TransformStamped

if __name__ == "__main__":
    rospy.init_node("laser_transformer")
    print("Laser Transformer Starting")
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transform = TransformStamped()

    static_transform.header.stamp = rospy.Time.now()
    static_transform.header.frame_id = "Robot"
    static_transform.child_frame_id = "laser"

    #Every other parameter can be 0 because the laser frame should match up with the Robot frame
    static_transform.transform.rotation.w = 1

    try:
        broadcaster.sendTransform(static_transform)
    except: 
        pass
    rospy.spin()