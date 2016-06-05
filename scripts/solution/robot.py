#!/usr/bin/env python
import rospy
import random
from astar import astar
from mdp import mdp
from read_config import read_config
from std_msgs.msg import Bool, String, Float32
from cse_190_assi_3.msg import *



class Robot():
	def __init__(self):
		"""Read config file and setup ROS things"""
		self.config = read_config()

		# Possible moves that can be made by robot: move_list
		self.move_list = self.config["move_list"]
		self.map_size = self.config["map_size"]

		# Create our robot node
		rospy.init_node("robot")

		self.finishPub = rospy.Publisher("/map_node/sim_complete", Bool, queue_size = 1)

		self.tasks()
		self.complete()

	def tasks(self):
		# A*
		intList = astar()

		# OUTPUT
		# Outputs the locations that must be traversed from the start to the goal including start and goal.
		aPub = rospy.Publisher("/results/path_list",AStarPath, queue_size = (self.map_size[0]*self.map_size[1]))
		for move in intList:
			aPath = AStarPath()
			aPath.data = move
			rospy.sleep(2)
			aPub.publish(aPath)

		# MDP
		mdp()



	def complete(self):
		rospy.sleep(2)
		self.finishPub.publish(Bool(True))
		rospy.sleep(2)
		rospy.signal_shutdown("robot")

if __name__ == '__main__':
	robot = Robot()