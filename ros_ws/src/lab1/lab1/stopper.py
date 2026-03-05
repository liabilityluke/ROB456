#!/usr/bin/env python3

# Bill Smart, smartw@oregonstate.edu
#
# stopper.py
# ASSIGNMENT: Slow the robot to a stop as it approaches a target


# Every Python node in ROS2 should include these lines.  rclpy is the basic Python
# ROS2 stuff, and Node is the class we're going to use to set up the node.
import rclpy
from rclpy.node import Node


# We're going to do some math
import numpy as np

# Header for the twist message
from std_msgs.msg import Header

# Velocity commands are given with Twist messages, from geometry_msgs
from geometry_msgs.msg import TwistStamped

# Laser scans are given with the LaserScan message, from sensor_msgs
from sensor_msgs.msg import LaserScan


class MyStopper(Node):
	def __init__(self):
		# Initialize the parent class, giving it a name.  The idiom is to use the
		# super() class.
		super().__init__('my_stopper')

		# Set up a publisher.  The default topic for Twist messages is cmd_vel.
		self.pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)

		# Set up a subscriber.  The default topic for LaserScan messages is base_scan.
		self.sub = self.create_subscription(LaserScan, 'base_scan', self.callback, 10)

		# GUIDE: Any variables that you want to add can go here
		self.robot_width = 0.38

	def callback(self, scan):
		# Every time we get a laser scan, calculate the shortest scan distance in front
		# of the robot, and set the speed accordingly.  We assume that the robot is 38cm
		# wide.  This means that y-values with absolute values greater than 19cm are not
		# in front of the robot.  It also assumes that the LiDAR is at the front of the
		# robot (which it actually isn't) and that it's centered and pointing forwards.
		# We can work around these assumptions, but it's cleaner if we don't

		# Pulling out some useful values from scan
		#   Start and top angles of the scan. 0 degrees is in front of the robot
		# Also given: Total number of scans in that range
		angle_min = scan.angle_min
		angle_max = scan.angle_max
		num_readings = len(scan.ranges)

		angles = np.linspace(angle_min, angle_max, num_readings)

		width = self.robot_width

		xs = np.zeros(num_readings)
		ys = np.zeros(num_readings)
		xs_in_front = []
		ys_in_front = []
		for i in range(num_readings) :
			xs[i] = scan.ranges[i] * np.cos(angles[i])
			ys[i] = scan.ranges[i] * np.sin(angles[i])
			if abs(ys[i]) < width/2 :
				xs_in_front.append(xs[i])
				ys_in_front.append(ys[i])
		
		xs_in_front = np.array(xs_in_front)
		ys_in_front = np.array(ys_in_front)

		closest_dist = np.min(xs_in_front)

		t = TwistStamped()
		t.header = Header()
		t.header.frame_id = 'base_link'  # Transform is in the robot's coordinate frame
		t.header.stamp = self.get_clock().now().to_msg()  # What time are we sending this?

		if closest_dist > 1.0:
			t.twist.linear.x = (closest_dist - 1)
		elif closest_dist < 1.0 :
			t.twist.linear.x = (closest_dist - 1)
		else :
			t.twist.linear.x = 0

		t.twist.linear.y = 0.0
		t.twist.linear.z = 0.0
		t.twist.angular.x = 0.0
		t.twist.angular.y = 0.0
		t.twist.angular.z = 0.0

		shortest = 0
		max_speed = 0.2

		self.pub.publish(t)

		self.get_logger().info(f'Shortest {shortest}, speed {t.twist.linear.x}')



# The idiom in ROS2 is to use a function to do all of the setup and work.  This
# function is referenced in the setup.py file as the entry point of the node when
# we're running the node with ros2 run.  The function should have one argument, for
# passing command line arguments, and it should default to None.
def main(args=None):
	# Initialize rclpy.  We should do this every time.
	rclpy.init(args=args)

	# Make a node class.  The idiom in ROS2 is to encapsulte everything in a class
	# that derives from Node.
	stopper = MyStopper()

	# The spin() call gives control over to ROS2, and it now takes a Node-derived
	# class as a parameter.
	rclpy.spin(stopper)

	# Make sure we shutdown everything cleanly.  This should happen, even if we don't
	# include this line, but you should do it anyway.
	rclpy.shutdown()
	

# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	# The idiom in ROS2 is to set up a main() function and to call it from the entry
	# point of the script.
	main()