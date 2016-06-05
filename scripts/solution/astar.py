# astar implementation needs to go here
import heapq
from read_config import read_config

def astar():
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
	mapSize = config["map_size"]

	# Location of the pits: pits
	pits = config["pits"]

	############
	root = Node(None, startLoc, None)


	visited = []

	# HEAPQ
	frontier = []
	heapq.heappush(frontier, (root.getHeuristic(), root))

	while(len(frontier) > 0):

		# Get tuple with (Heuristic#, Node)
		current = (heapq.heappop(frontier)) [1]
		currentState = current.getState()

		visited.append(currentState)

		#Check if goal is reached
		if(currentState == goalLoc):
			#print 'Goal state reached'
			toReturn = []
			while (current.getAction() != None):
				toReturn.append(current.getState())
				current = current.parent
			toReturn.append(current.getState())
			#print toReturn[::-1]
			return toReturn[::-1]


		# Add each move into frontier if the transition state is valid
		for move in move_list:
			newState = [currentState[0]+move[0], currentState[1]+move[1]]
			valid = True

			# Already expanded state
			for elem in visited:
				if(newState == elem):
					valid = False
					continue

			# Check for walls
			for elem in walls:
				if(newState == elem):
					valid = False
					continue

			# Avoid pits
			for elem in pits:
				if(newState == elem):
					valid = False
					continue

			# X Boundaries
			if (newState[0] == (mapSize[0]+1) or newState[0] == -1):
				valid = False
			# Y Boundaries
			if (newState[1] == (mapSize[1]+1) or newState[1] == -1):
				valid = False

			if valid:
				newNode = Node(current, newState, move)
				heapq.heappush(frontier, (newNode.getHeuristic(), newNode))




class Node:
	def __init__ (self, parent, state, action):
		self.parent = parent
		self.state = state
		self.action = action

	def getHeight(self):
		i = 0
		cur = self
		while (cur.parent != None):
			i+=1
			cur = cur.parent
		return i

	# Manhattan distance
	def getHeuristic(self):
		config = read_config()
		goalLoc = config["goal"]
		manDist = abs(goalLoc[0] - self.state[0]) + abs(goalLoc[1] - self.state[1])
		return (manDist + self.getHeight())

	################# Get Commands #####################
	def getState(self):
		return self.state

	def isEqual (self, state):
		return self.state == state

	def getAction(self):
		return self.action

	