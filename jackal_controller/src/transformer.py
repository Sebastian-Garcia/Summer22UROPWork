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

        #Subscribes to the topic with the goal pose relative to the world frame
        self.goal_sub = rospy.Subscriber("/goal", PoseStamped, self.goal_cb)

        #Publishing to a new topic with the goal pose relative to the CameraStand frame or Robot
        self.relative_pub = rospy.Publisher("/relative_goal", PoseStamped, queue_size=10)
        
        #Subscribes to pose from robot front plate to then get the center of the robot
        self.front_base_sub = rospy.Subscriber("/vrpn_client_node/Robot/pose", PoseStamped, self.front_base_cb)

        
    
    def goal_cb(self, data):

        #Tries to get a transform to the target frame of the robot,
        #Then transforms the pose into it and publishes it to the new topic 
        try:
            transform = self.tfBuffer.lookup_transform("base_link", data.header.frame_id, data.header.stamp, rospy.Duration(1.0))
            pose_transformed = tf2_geometry_msgs.do_transform_pose(data, transform)
            self.relative_pub.publish(pose_transformed)
        except:
            print("hello, reached here!")
            pass

    def front_base_cb(self, data):
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
    
    

if __name__ == "__main__":
    try:
        rospy.init_node("Transformer")
        Transformer()
        print("Starting Transformer")

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
