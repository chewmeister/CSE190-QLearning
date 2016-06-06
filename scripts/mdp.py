# mdp implementation needs to go here
import rospy
from read_config import read_config
from copy import deepcopy
from cse_190_assi_fpa.msg import *


def mdp():
	# All action outcomes, both favorable and unfavorable, 
	#	have rewards (positive/negative) and they're provided for you.
	# INPUT
	config = read_config()
	# Possible moves that can be made by robot: move_list
	move_list = config["move_list"]

	# Start location of the robot and goal location: start and goal
	startLoc = config["start"]
	goalLoc = config["goal"]

	# Location of obstacles and walls.
	# Walls are explicitly defined as occupied cells along the boundaries of the map.
	# If you attempt to go past the edge of a map, you assume that you'd hit a wall.
	# Use parameter: wall in json file
	walls = config["walls"]

	# Location of the pits: pits
	pits = config["pits"]

	map_size = config["map_size"]

	###################

	# Probabilistic Model for the outcomes of the robot's actions. 
	# p_move_forward, p_move_backward, p_move_left, p_move_right. 

	# Action outcomes and rewards.
	# Pits and goals are absorbing state; robot cannot move out of these states.
	# Robot gets a reward after transitioning to these states and no additional future rewards.

	# In config file, the rewards are given by:
	rewardPit = config["reward_for_falling_in_pit"]
	rewardGoal = config["reward_for_reaching_goal"]

	# Any action that results in a change in the robot's position costs the robot a negative reward
	rewardStep = config["reward_for_each_step"]

	# If the robot collides with a wall, it will remain at the current location
	# It'll get a negative reward for hitting the wall and will receive all discounted rewards from that state.
	rewardWall = config["reward_for_hitting_wall"]

	# Robot doesn't incur the cost for movement if colliding with awall.

	# Forward is another word for going the right direction
	pForward = config["prob_move_forward"]
	pBackward = config["prob_move_backward"]
	pLeft = config["prob_move_left"]
	pRight = config["prob_move_right"]

	maxIterations = config["max_iterations"]
	thresholdDiff = config["threshold_difference"]

	discountFactor = config["discount_factor"]

	# OUTPUT
	# Representing policies, use value iteration to compute the values for each cell in the map
	# Extract the optimal policy after each iteration
	# Policy should be expressed as a list of strings each taking one of the following values
	# WALL -  This cell is a wall
	# GOAL - This cell is the goal
	# PIT - This cell is a pit
	# N - The robot moves in the direction of North from this cell
	# S - The robot moves in the direction of South from this cell
	# W - The robot moves in the direction of West from this cell
	# E - The robot moves in the direction of East from this cell

	# String list
	policyPub = rospy.Publisher("/results/policy_list", PolicyList, queue_size = map_size[0]*map_size[1])

	values = {}
	update = {}
	for i in range(0,map_size[0]):
		for j in range(0,map_size[1]):
			values[(i,j)] = Node([i,j])
			update[(i,j)] = Node([i,j])

	returnArray = []

	iterationCount = 0;
	while(iterationCount != maxIterations):
		diff = 0.0
		returnArray = []

		#print "Iteration: " + str(iterationCount) + " ----------------------------"

		for i in range(0,map_size[0]):
			for j in range(0,map_size[1]):
				state = (i,j)
				node = values[state]

				if node.isValid():
					################ North ################
					# Forward
					newState = [state[0] - 1, state[1] + 0]
					if (newState[0] == -1 or newState[0] == map_size[0] or newState[1] == -1 or newState[1] == map_size[1]):
						tempForward = pForward*(discountFactor*values[state].getValue() + rewardWall)
					else:
						(x,y) = newState
						nodeDirection = values[(x,y)]
						if(nodeDirection.boolWall()):
							tempForward = pForward*(discountFactor*values[state].getValue() + rewardWall)
						else:
							tempForward = pForward*(discountFactor*nodeDirection.getValue() + rewardStep)

					# Backward
					newState = [state[0] + 1, state[1] + 0]
					if (newState[0] == -1 or newState[0] == map_size[0] or newState[1] == -1 or newState[1] == map_size[1]):
						tempBackward = pBackward*(discountFactor*values[state].getValue() + rewardWall)
					else:
						(x,y) = newState
						nodeDirection = values[(x,y)]
						if(nodeDirection.boolWall()):
							tempBackward = pBackward*(discountFactor*values[state].getValue() + rewardWall)
						else:
							tempBackward = pBackward*(discountFactor*nodeDirection.getValue() + rewardStep)


					# Right
					newState = [state[0] + 0, state[1] + 1]
					if (newState[0] == -1 or newState[0] == map_size[0] or newState[1] == -1 or newState[1] == map_size[1]):
						tempRight = pRight*(discountFactor*values[state].getValue() + rewardWall)
					else:
						(x,y) = newState
						nodeDirection = values[(x,y)]
						if(nodeDirection.boolWall()):
							tempRight = pRight*(discountFactor*values[state].getValue() + rewardWall)
						else:
							tempRight = pRight*(discountFactor*nodeDirection.getValue() + rewardStep)


					# Left
					newState = [state[0] + 0, state[1] - 1]
					if (newState[0] == -1 or newState[0] == map_size[0] or newState[1] == -1 or newState[1] == map_size[1]):
						tempLeft = pLeft*(discountFactor*values[state].getValue() + rewardWall)
					else:
						(x,y) = newState
						nodeDirection = values[(x,y)]
						if(nodeDirection.boolWall()):
							tempLeft = pLeft*(discountFactor*values[state].getValue() + rewardWall)
						else:
							tempLeft = pLeft*(discountFactor*nodeDirection.getValue() + rewardStep)


					newUtility = (tempForward + tempBackward + tempRight + tempLeft)
					update[state].setQVal("N", newUtility)

					################ South ################
					# Forward
					newState = [state[0] + 1, state[1] + 0]
					if (newState[0] == -1 or newState[0] == map_size[0] or newState[1] == -1 or newState[1] == map_size[1]):
						tempForward = pForward*(discountFactor*values[state].getValue() + rewardWall)
					else:
						(x,y) = newState
						nodeDirection = values[(x,y)]
						if(nodeDirection.boolWall()):
							tempForward = pForward*(discountFactor*values[state].getValue() + rewardWall)
						else:
							tempForward = pForward*(discountFactor*nodeDirection.getValue() + rewardStep)

					# Backward
					newState = [state[0] - 1, state[1] + 0]
					if (newState[0] == -1 or newState[0] == map_size[0] or newState[1] == -1 or newState[1] == map_size[1]):
						tempBackward = pBackward*(discountFactor*values[state].getValue() + rewardWall)
					else:
						(x,y) = newState
						nodeDirection = values[(x,y)]
						if(nodeDirection.boolWall()):
							tempBackward = pBackward*(discountFactor*values[state].getValue() + rewardWall)
						else:
							tempBackward = pBackward*(discountFactor*nodeDirection.getValue() + rewardStep)


					# Right
					newState = [state[0] + 0, state[1] - 1]
					if (newState[0] == -1 or newState[0] == map_size[0] or newState[1] == -1 or newState[1] == map_size[1]):
						tempRight = pRight*(discountFactor*values[state].getValue() + rewardWall)
					else:
						(x,y) = newState
						nodeDirection = values[(x,y)]
						if(nodeDirection.boolWall()):
							tempRight = pRight*(discountFactor*values[state].getValue() + rewardWall)
						else:
							tempRight = pRight*(discountFactor*nodeDirection.getValue() + rewardStep)


					# Left
					newState = [state[0] + 0, state[1] + 1]
					if (newState[0] == -1 or newState[0] == map_size[0] or newState[1] == -1 or newState[1] == map_size[1]):
						tempLeft = pLeft*(discountFactor*values[state].getValue() + rewardWall)
					else:
						(x,y) = newState
						nodeDirection = values[(x,y)]
						if(nodeDirection.boolWall()):
							tempLeft = pLeft*(discountFactor*values[state].getValue() + rewardWall)
						else:
							tempLeft = pLeft*(discountFactor*nodeDirection.getValue() + rewardStep)

					newUtility = (tempForward + tempBackward + tempRight + tempLeft)
					update[state].setQVal("S", newUtility)

					################ West ################
					# Forward
					newState = [state[0] + 0, state[1] - 1]
					if (newState[0] == -1 or newState[0] == map_size[0] or newState[1] == -1 or newState[1] == map_size[1]):
						tempForward = pForward*(discountFactor*values[state].getValue() + rewardWall)
					else:
						(x,y) = newState
						nodeDirection = values[(x,y)]
						if(nodeDirection.boolWall()):
							tempForward = pForward*(discountFactor*values[state].getValue() + rewardWall)
						else:
							tempForward = pForward*(discountFactor*nodeDirection.getValue() + rewardStep)

					# Backward
					newState = [state[0] + 0, state[1] + 1]
					if (newState[0] == -1 or newState[0] == map_size[0] or newState[1] == -1 or newState[1] == map_size[1]):
						tempBackward = pBackward*(discountFactor*values[state].getValue() + rewardWall)
					else:
						(x,y) = newState
						nodeDirection = values[(x,y)]
						if(nodeDirection.boolWall()):
							tempBackward = pBackward*(discountFactor*values[state].getValue() + rewardWall)
						else:
							tempBackward = pBackward*(discountFactor*nodeDirection.getValue() + rewardStep)


					# Right
					newState = [state[0] - 1, state[1] + 0]
					if (newState[0] == -1 or newState[0] == map_size[0] or newState[1] == -1 or newState[1] == map_size[1]):
						tempRight = pRight*(discountFactor*values[state].getValue() + rewardWall)
					else:
						(x,y) = newState
						nodeDirection = values[(x,y)]
						if(nodeDirection.boolWall()):
							tempRight = pRight*(discountFactor*values[state].getValue() + rewardWall)
						else:
							tempRight = pRight*(discountFactor*nodeDirection.getValue() + rewardStep)


					# Left
					newState = [state[0] + 1, state[1] + 0]
					if (newState[0] == -1 or newState[0] == map_size[0] or newState[1] == -1 or newState[1] == map_size[1]):
						tempLeft = pLeft*(discountFactor*values[state].getValue() + rewardWall)
					else:
						(x,y) = newState
						nodeDirection = values[(x,y)]
						if(nodeDirection.boolWall()):
							tempLeft = pLeft*(discountFactor*values[state].getValue() + rewardWall)
						else:
							tempLeft = pLeft*(discountFactor*nodeDirection.getValue() + rewardStep)

					newUtility = (tempForward + tempBackward + tempRight + tempLeft)
					update[state].setQVal("W", newUtility)

					################ East ################
					# Forward
					newState = [state[0] + 0, state[1] + 1]
					if (newState[0] == -1 or newState[0] == map_size[0] or newState[1] == -1 or newState[1] == map_size[1]):
						tempForward = pForward*(discountFactor*values[state].getValue() + rewardWall)
					else:
						(x,y) = newState
						nodeDirection = values[(x,y)]
						if(nodeDirection.boolWall()):
							tempForward = pForward*(discountFactor*values[state].getValue() + rewardWall)
						else:
							tempForward = pForward*(discountFactor*nodeDirection.getValue() + rewardStep)

					# Backward
					newState = [state[0] + 0, state[1] - 1]
					if (newState[0] == -1 or newState[0] == map_size[0] or newState[1] == -1 or newState[1] == map_size[1]):
						tempBackward = pBackward*(discountFactor*values[state].getValue() + rewardWall)
					else:
						(x,y) = newState
						nodeDirection = values[(x,y)]
						if(nodeDirection.boolWall()):
							tempBackward = pBackward*(discountFactor*values[state].getValue() + rewardWall)
						else:
							tempBackward = pBackward*(discountFactor*nodeDirection.getValue() + rewardStep)


					# Right
					newState = [state[0] + 1, state[1] + 0]
					if (newState[0] == -1 or newState[0] == map_size[0] or newState[1] == -1 or newState[1] == map_size[1]):
						tempRight = pRight*(discountFactor*values[state].getValue() + rewardWall)
					else:
						(x,y) = newState
						nodeDirection = values[(x,y)]
						if(nodeDirection.boolWall()):
							tempRight = pRight*(discountFactor*values[state].getValue() + rewardWall)
						else:
							tempRight = pRight*(discountFactor*nodeDirection.getValue() + rewardStep)


					# Left
					newState = [state[0] - 1, state[1] + 0]
					if (newState[0] == -1 or newState[0] == map_size[0] or newState[1] == -1 or newState[1] == map_size[1]):
						tempLeft = pLeft*(discountFactor*values[state].getValue() + rewardWall)
					else:
						(x,y) = newState
						nodeDirection = values[(x,y)]
						if(nodeDirection.boolWall()):
							tempLeft = pLeft*(discountFactor*values[state].getValue() + rewardWall)
						else:
							tempLeft = pLeft*(discountFactor*nodeDirection.getValue() + rewardStep)

					newUtility = (tempForward + tempBackward + tempRight + tempLeft)
					update[state].setQVal("E", newUtility)

					# Update the policy and values!
					
					update[state].setPolicy()
					# print values[state].getValue()
					# print update[state].getValue()
					diff += abs(values[state].getValue() - update[state].getValue())
				# print "^State: " + str(state[0])  + ", " + str(state[1]) + "\n"
				#print "Old value: " + str(node.getValue())
				#print "New value: " + str(update[state].getValue()) + "\n"
				returnArray.append(update[state].getPolicy())


		# Check the threshold_difference between current iteration and previous iteration
		# If absolute sum of differences is less than threshold_difference
		toPublish = PolicyList()
		#print returnArray
		toPublish.data = returnArray
		policyPub.publish(toPublish.data)
		rospy.sleep(2)

		#print diff
		if diff <= thresholdDiff:
			break

		# if iterationCount == 1:
		# 	break

		for i in range(0,map_size[0]):
			for j in range(0,map_size[1]):
				if(values[(i,j)].isValid()):
					values[(i,j)].setQVal("N",update[(i,j)].getQVal("N"))
					values[(i,j)].setQVal("S",update[(i,j)].getQVal("S"))
					values[(i,j)].setQVal("E",update[(i,j)].getQVal("E"))
					values[(i,j)].setQVal("W",update[(i,j)].getQVal("W"))
					values[(i,j)].setPolicy()
		iterationCount += 1
	return

# u(s) = r(s) + gamma * max_a Sig (T(s, a, s') u(s'))
class Node:
	def __init__ (self, state):
		config = read_config()
		goalLoc = config["goal"]
		walls = config["walls"]
		pits = config["pits"]
		rewardPit = config["reward_for_falling_in_pit"]
		rewardGoal = config["reward_for_reaching_goal"]
		rewardWall = config["reward_for_hitting_wall"]

		self.state = state
		self.canAlter = True
		self.isWall = False

		self.policy = None
		self.value = 0
		# Left, Right, Up, Down
		self.qVals = {"W": 0, "E": 0, "N": 0, "S": 0}


		if (state == goalLoc):
			self.policy = "GOAL"
			self.value = rewardGoal
			self.canAlter = False
		else:
			for pit in pits:
				if (state == pit):
					self.policy = "PIT"
					self.value = rewardPit
					self.canAlter = False

		for wall in walls:
			if (state == wall):
				self.policy = "WALL"
				self.value = rewardWall
				self.canAlter = False
				self.isWall = True

	################# Get Commands #####################

	def getState(self):
		return self.state

	def getQVal(self, direction):
		return self.qVals[direction]

	def setQVal(self, direction, value):
		self.qVals[direction] = value

	def setPolicy(self):
		maxValue = float("-inf")
		bestPolicy = None
		# print "Setting Policy -------------"
		for direction, value in self.qVals.iteritems():
			# print direction + ": " + str(value) + "\n "
			if(maxValue < value):
				maxValue = value
				bestPolicy = direction

		self.value = maxValue
		self.policy = bestPolicy

		# print "Policy: " + str(self.policy)
		# print "Value: " + str(self.value)


	def getPolicy(self):
		return self.policy

	def getValue(self):
		return self.value

	def boolWall(self):
		return self.isWall

	def isValid(self):
		return self.canAlter

	def isEqual (self, state):
		return self.state == state



	