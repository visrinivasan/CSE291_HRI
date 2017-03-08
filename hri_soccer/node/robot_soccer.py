#!/usr/bin/env python
import rospy
import cv_bridge
import cv
import sensor_msgs.msg as sm
from std_msgs.msg import String


class SoccerPlayer(object):
	def __init__(self):
		""" Initializes soccer player object to not running. """
		self.running = False
		self.current_state = "Wait"	# Starts game waiting for task command


	def wait_for_voice_input():
		"""Waits for user to command robot to do a task."""
		pass

	def run(self):
		""" Runs robot soccer application. """

		rospy.init_node('robot_soccer')
		pub = rospy.Publisher('my_task', String, queue_size=10)
	
		# Start
		self.running = True

		rate = rospy.Rate(10)	# 10 Hz

		while not rospy.is_shutdown():
			if self.running:
				pub.publish(self.current_state)
				# Get voice input here
			rate.sleep()



if __name__ == "__main__":
	try:
		robot_soccer = SoccerPlayer()
		robot_soccer.run()
	except rospy.ROSInterruptException:
		pass
