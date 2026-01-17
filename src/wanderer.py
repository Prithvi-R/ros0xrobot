#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud

class TurtleWanderer:
	def __init__(self):
		rospy.init_node('turtle_wanderer')
		
		self.pub = rospy.Publisher('ros0xrobot/cmd_vel',Twist,queue_size=10)
		self.sub = rospy.Subscriber('ros0xrobot/sonar',PointCloud,self.sonar_callback)
		self.move_cmd = Twist()
		self.obstacle_detected = False
		self.state = "FORWARD"
		
		self.safe_distance = 450.0
	
	def sonar_callback(self,data):
		"""
		Fire Bird VI has 8 sonar sensor.
		Point 3 and 4 are front facing sensors which will help in obstacle avoidance.
		"""
		if len(data.points) > 4:
			s2 = data.points[2].x
			s3 = data.points[3].x
			s4 = data.points[4].x
			
			left_obstacle = s4+s3 < 0.15
			right_obstacle =  s2+s3 < 0.15
			front_obstacle = s2+s4 <0.3
			
			if left_obstacle:
				self.state = "Turn_LEFT"
			elif right_obstacle:
				self.state = "Turn_RIGHT"
			elif front_obstacle:
				self.state = "BACKWARD"
			else:
				self.state = "FORWARD"
			
	def start_wandering(self):
		rate = rospy.Rate(10)
		rospy.loginfo("Turtle Mode Activated: Wandering effortlessly...")
		
		while not rospy.is_shutdown():
			if self.state == "Turn_RIGHT":
				rospy.loginfo("Rotating Left effortlessly...")
				self.move_cmd.linear.x = 0.0
				self.move_cmd.angular.z = -0.5
			elif self.state == "Turn_LEFT":
				rospy.loginfo("Rotating Right effortlessly...")
				self.move_cmd.linear.x = 0.0
				self.move_cmd.angular.z = 0.5
			elif self.state == "BACKWARD":
				rospy.loginfo("Turning Back effortlessly...")
				self.move_cmd.linear.x = -0.2
				self.move_cmd.angular.z = 0.1
				
			else:
				self.move_cmd.linear.x = 0.1
				self.move_cmd.angular.z = 0.0
			
			self.pub.publish(self.move_cmd)
			rate.sleep()
			
if __name__ == '__main__':
	try:
		turtle = TurtleWanderer()
		turtle.start_wandering()
	except rospy.ROSInterruptException:
		stop_cmd = Twist()
		turtle.pub.publish(stop_cmd)
