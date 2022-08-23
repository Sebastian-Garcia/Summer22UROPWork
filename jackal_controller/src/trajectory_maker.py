#!/usr/bin/env python3


import rospy
import numpy as np
import numpy.linalg as LA
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool


#This class will set a arbitrary "goal" poses after the robot
#has successfully traversed to one in sequential order
class TrajectoryMaker():
    def __init__(self) -> None:
        self.pose_coords = [(1.144,0.554), (1.361,0.214),(0.994, -.903), (0.466,-1.225),
                            (-0.196,-1.061),(-0.368,0.047),(-0.141, 0.738),(0.430,1.055),(1.206,0.601)]
        
        self.pose_coords = self.generate_circle_trajectory((0.1925, 0.1), 1.1)
        print(self.pose_coords)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped,queue_size=10)
        self.stopped_sub = rospy.Subscriber("/stopped", Bool, self.start_cb)
        self.index = 0

    def start_cb(self,data):
        #If no more poses left, then just stop the function here
        rospy.sleep(.1)
        if self.index >= len(self.pose_coords):
            rospy.signal_shutdown("ended trajectory")

        coord = self.pose_coords[self.index]
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.pose.position.x = coord[0]
        msg.pose.position.y = coord[1]
        msg.pose.orientation.w = 1
        self.goal_pub.publish(msg)

        #Increment index to move on to next pose when finished
        self.index += 1


    def generate_circle_trajectory(self, center, radius):
        numPoints = 16
        angles = [x * 2*np.pi/numPoints for x in range(numPoints + 1)] #angles in radians
        coords = [(np.cos(angle) + center[0], np.sin(angle)+center[1]) for angle in angles]
        return coords




        print("Starting TrajectoryMaker")
        rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node("TrajectoryMaker")
        tm = TrajectoryMaker()
        print("Starting TrajectoryMaker")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass