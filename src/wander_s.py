#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class TurtleWanderer:
    def __init__(self):
        rospy.init_node('turtle_wanderer')
        self.pub = rospy.Publisher('ros0xrobot/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        
        self.move_cmd = Twist()
        self.state = "FORWARD"
        self.safe_distance = 0.6  # Increased slightly for safety
        
        # Persistence Logic
        self.turn_timer = 0
        self.min_turn_duration = 5 # Number of loops to stay in "Turn" state (at 10Hz, 20 = 2 seconds)

    def scan_callback(self, data):
        ranges = np.array(data.ranges)
        ranges[ranges < 0.01] = 10.0 # Clean bad data
        
        # Check a wider front arc (±45 degrees)
        front_arc = np.concatenate((ranges[0:45], ranges[315:360]))
        min_dist = min(front_arc)

        # If we are already in a "Forced Turn," don't let the Lidar interrupt it
        if self.turn_timer > 0:
            self.turn_timer -= 1
            return

        if min_dist < self.safe_distance:
            rospy.loginfo("Obstacle! Starting forced turn...")
            self.state = "RECOVER_TURN"
            self.turn_timer = self.min_turn_duration # Start the 2-second timer
        else:
            self.state = "FORWARD"

    def start_wandering(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.state == "RECOVER_TURN":
                # Rotate sharply while the timer is active
                self.move_cmd.linear.x = 0.0
                self.move_cmd.angular.z = 0.7 
            else:
                # Normal forward movement
                self.move_cmd.linear.x = 0.15 
                self.move_cmd.angular.z = 0.0

            self.pub.publish(self.move_cmd)
            rate.sleep()

if __name__ == '__main__':
    try:
        turtle = TurtleWanderer()
        turtle.start_wandering()
    except rospy.ROSInterruptException:
        pass