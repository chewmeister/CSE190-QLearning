# qlearning implementation goes here
import rospy
import random as r
from copy import deepcopy
from read_config import read_config
from cse_190_assi_fpa.msg import *

def flipCoin( p ):
    rand = r.random()
    return rand < p

#(1-a)*Q(A,Right) + a(R(A, Right, S') + d*max_a(Q(S',A)))
def qlearnNoMoveProb():
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

	qDiscount = config["qDiscount"]
	qAlpha = config["qAlpha"]
	qEpsilon = config["qEpsilon"]

	# String list
	policyPub = rospy.Publisher("/results/policy_list", PolicyList, queue_size = map_size[0]*map_size[1])

	values = {}
	update = {}
	for i in range(0,map_size[0]):
		for j in range(0,map_size[1]):
			values[(i,j)] = Node([i,j])
			update[(i,j)] = Node([i,j])

	returnArray = []

	start = r.choice(list(values.values()))
	while(not start.isValid()):
		start = r.choice(list(values.values()))

	curr = start
	stepNum = 0

	moves = ["W","N","S","E"]

	iterationCount = 0;
	while(iterationCount != maxIterations):
		diff = 0.0
		returnArray = []

		while(not curr.isValid()):
			curr = r.choice(list(values.values()))

		state = curr.getState()
		nextState = (None,None)


		# Get random direction based on pro
		if flipCoin(qEpsilon):	
			randMove = r.choice(moves)
		else: 
			randMove = curr.getPolicy()

		################ North ################
		if( randMove == "N"):
			
			# Forward
			newState = [state[0] - 1, state[1] + 0]
			if (newState[0] == -1 or newState[0] == map_size[0] or newState[1] == -1 or newState[1] == map_size[1]):
				# You've hit a wall and you're back at your own tile.
				maxQNext = rewardWall + curr.getValue()
				(x,y) = state
				nextState = state
			else:
				(x,y) = newState
				nodeDirection = values[(x,y)]
				if(nodeDirection.boolWall()):
					maxQNext = rewardWall + (qDiscount*curr.getValue())
					nextState = state
				else:
					maxQNext = rewardStep + (qDiscount*values[(x,y)].getValue())
					nextState = newState
			
			# Backward
			newState = [state[0] + 1, state[1] + 0]	

			# Right
			newState = [state[0] + 0, state[1] + 1]

			# Left
			newState = [state[0] + 0, state[1] - 1]

		################ South ################
		if( randMove == "S"):
			newState = [state[0] + 1, state[1] + 0]
			if (newState[0] == -1 or newState[0] == map_size[0] or newState[1] == -1 or newState[1] == map_size[1]):
				# You've hit a wall and you're back at your own tile.
				maxQNext = rewardWall + curr.getValue()
				nextState = state
			else:
				(x,y) = newState
				nodeDirection = values[(x,y)]
				if(nodeDirection.boolWall()):
					maxQNext = rewardWall + (qDiscount*curr.getValue())
					nextState = state
				else:
					maxQNext = rewardStep + (qDiscount*values[(x,y)].getValue())
					nextState = newState

			# Backward
			newState = [state[0] - 1, state[1] + 0]
			# Right
			newState = [state[0] + 0, state[1] - 1]
			# Left
			newState = [state[0] + 0, state[1] + 1]

		################ East ################
		if( randMove == "E"):
			newState = [state[0] + 0, state[1] + 1]
			if (newState[0] == -1 or newState[0] == map_size[0] or newState[1] == -1 or newState[1] == map_size[1]):
				# You've hit a wall and you're back at your own tile.
				maxQNext = rewardWall + curr.getValue()
				nextState = state
			else:
				(x,y) = newState
				nodeDirection = values[(x,y)]
				if(nodeDirection.boolWall()):
					maxQNext = rewardWall + (qDiscount*curr.getValue())
					nextState = state
				else:
					maxQNext = rewardStep + (qDiscount*values[(x,y)].getValue())
					nextState = newState

			# Backward
			newState = [state[0] + 0, state[1] - 1]
			# Right
			newState = [state[0] + 1, state[1] + 0]
			# Left
			newState = [state[0] - 1, state[1] + 0]

		################ West ################
		if( randMove == "W"):
			newState = [state[0] + 0, state[1] - 1]
			if (newState[0] == -1 or newState[0] == map_size[0] or newState[1] == -1 or newState[1] == map_size[1]):
				# You've hit a wall and you're back at your own tile.
				maxQNext = rewardWall + curr.getValue()
				nextState = state
			else:
				(x,y) = newState
				nodeDirection = values[(x,y)]
				if(nodeDirection.boolWall()):
					maxQNext = rewardWall + (qDiscount*curr.getValue())
					nextState = state
				else:
					maxQNext = rewardStep + (qDiscount*values[(x,y)].getValue())
					nextState = newState
			# Backward
			newState = [state[0] + 0, state[1] + 1]


			# Right
			newState = [state[0] - 1, state[1] + 0]


			# Left
			newState = [state[0] + 1, state[1] + 0]



		valueForMove = (1-qAlpha)*curr.getQVal(randMove) + qAlpha*maxQNext
		curr.setQVal(randMove,valueForMove)
		print "current state: " + str(curr.getState())
		print "iter: " + str(iterationCount) + ", Looked at State: " + str(nextState)
		print "qVal: " + randMove + ", value: " + str(valueForMove) + "\n"
		curr.setPolicy()

		(x,y) = nextState
		curr = values[(x,y)]
		stepNum += 1

		# Reset the state if it's somewhere invalid (Goal/Pit)
		while(not curr.isValid()):
			curr = r.choice(list(values.values()))
			stepNum = 0

		print "Next current value: " + str(curr.getState()) + "****** \n"

		for i in range(0,map_size[0]):
			for j in range(0,map_size[1]):
				returnArray.append(values[(i,j)].getPolicy())
		toPublish = PolicyList()
		#print returnArray
		toPublish.data = returnArray
		policyPub.publish(toPublish.data)
		rospy.sleep(2)

		iterationCount += 1


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

		self.policy = "W"
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