#!/usr/bin/env python3


import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import tf_conversions
from geometry_msgs.msg import PoseStamped, TransformStamped

class Transformer():

    def __init__(self) -> None:


        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.br = tf2_ros.TransformBroadcaster()

    def base_camera_cb(self, data):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "Robot"
        t.child_frame_id = "base_link"
        t.transform.translation.x = -0.105 #Subtract 1-.5 centimeters in x-direction
        # t.transform.translation.y = data.pose.position.y
        # t.transform.translation.z = data.pose.position.z
        # t.transform.rotation.x = data.pose.orientation.x
        # t.transform.rotation.y = data.pose.orientation.y
        # t.transform.rotation.z = data.pose.orientation.z
        # t.transform.rotation.w = data.pose.orientation.w
        t.transform.rotation.w = 1
        self.br.sendTransform(t)