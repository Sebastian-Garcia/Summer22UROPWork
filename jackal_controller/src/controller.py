#!/usr/bin/env python3

import rospy
import numpy as np
import numpy.linalg as LA
from geometry_msgs.msg import Twist, PoseStamped, PoseArray
from std_msgs.msg import Bool

class Controller():

    def __init__(self) -> None:
        self.x_goal = 0
        self.y_goal = 0
        self.drive_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.relative_goal_sub = rospy.Subscriber("/relative_goal", PoseStamped, self.relative_goal_callback)
        self.stopped_pub = rospy.Publisher("/stopped", Bool, queue_size=10)
        self.rate = rospy.Rate(10)
        self.KP = 1.5 #this works #1.25 works, add some D
        self.KD = 0.1 #originally .50
        self.default_speed = .075
        self.default_angle = 0
        self.distance_threshold = 0.15
        self.angle_threshold = .13
        self.last_time = rospy.Time.now()
        self.last_error = 0
        self.alpha = 0.05
        self.last_derivative = 0

        self.goal_pose = None

    def drive_in_circle(self):
        while not rospy.is_shutdown():
            msg = Twist()
            msg.linear.x = .2
            msg.angular.z = -0.3
            self.drive_pub.publish(msg)
            self.rate.sleep()


    def relative_goal_callback(self, data):

        #In a loop, get target point, calculate distance error, and angle error as two
        #separate PID controls, and have the robot navigate respectively
            
        #Get goal location with respect to robot (camera stand)
        goal_x = data.pose.position.x
        goal_y = data.pose.position.y
        goal_z = data.pose.position.z

        print("X: " + str(goal_x) + "Y: " + str(goal_y))
        #Create new message for driving controls
        msg = Twist()
        speed = self.default_speed #velocity
        angle = 0 #angle (left is positive, right is negative)


        #PD Controls
        currTime = rospy.Time.now()
        angleToGoal = np.arctan2(goal_y, goal_x)
        error = angleToGoal

        dt = (currTime.to_sec() - self.last_time.to_sec())
        de = self.KD * (error - self.last_error)/dt

        P = self.KP*error
        D = self.alpha*de + (1-self.alpha)*self.last_derivative
        
        angle = P + D
        self.last_derivative = D
        self.last_error = error 
        self.last_time = currTime

        #Calculate distance error (will be about 0 when robot is on the goal point)
        distance_error = np.sqrt(goal_x**2 + goal_y**2)
        

        if self.facingGoal(goal_x, goal_y):
            angle = 0.0
            #print("facing goal")
            #If robot is close enough to goal, then stop
            if distance_error <= self.distance_threshold:
                speed = 0.0
                print("Stopping now!")

                #Now that the robot is positioned on the goal, we need to rotate it
                #To match the correct orientation of the goal pose, so that we can repeat experiments every time. 
                #Publish a singular message to another topic/node, and have it rotate until it's in the correct position. 
                self.stopped_pub.publish(True)

                #Shutdown this node so the robot does not move unwantingly.
                #rospy.signal_shutdown("Stopping Controller Now That Robot is In Position")
                

        else:
            #If not facing the cone but is on the correct spot, then just turn the robot
            if distance_error<= self.distance_threshold:
                speed = 0.0
                #self.stopped_pub.publish(True)
                #self.stopped_pub.publish(True)
        

        #Set speed and angle in message, and publish
        msg.linear.x = self.default_speed # min(speed * distance_error * 2, self.default_speed)
        msg.angular.z = angle
        self.drive_pub.publish(msg)

    def facingGoal(self,cone_x, cone_y):
        angle = np.arctan2(cone_y, cone_x)
        #print("At angle: " + str(angle) + " from goal point")
        return np.abs(angle) <= self.angle_threshold
    


if __name__ == '__main__':
    try:
        rospy.init_node("Controller")
        c = Controller()
        print("Starting Controller")
        #c.drive_in_circle()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass