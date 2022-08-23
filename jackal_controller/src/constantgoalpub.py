#!/usr/bin/env python3


import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class ConstantPub():
    def __init__(self) -> None:
        self.constant_pub = rospy.Publisher("/goal", PoseStamped, queue_size=10)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb)
        self.stopped_sub = rospy.Subscriber("/stopped", Bool, self.stop_cb)
        self.rate = rospy.Rate(10)
        self.stop = False

    #Takes a goal that was published once, and continously publishes it until this node is stopped
    def goal_cb(self, data):
        while not rospy.is_shutdown():
            if self.stop:
                print("reached here")
                break
            data.header.stamp = rospy.Time.now()
            self.constant_pub.publish(data)
            self.rate.sleep()
        
        self.stop = False
        print("Stopping Constant Pub")

    def stop_cb(self, data):
        if data:
            self.stop = True

if __name__ == "__main__":
    try:
        rospy.init_node("ConstantPub")
        c = ConstantPub()
        print("Starting Constant Goal Publisher")
        rospy.spin()
    except:
        pass
            