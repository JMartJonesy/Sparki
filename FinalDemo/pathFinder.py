"""
	Path Finder
"""

from math import sqrt, atan2, pi, sin, cos
from time import sleep
from serial import Serial
from collections import deque
from struct import pack, unpack

#graph = []
graph = [[1], [0]]
landMarks = [[77.77, -62.78], [107.35, -158.22]]

portName = "/dev/cu.ArcBotics-DevB-1"
serialPort = Serial()

#Connects to Sparki
def connect():
	print("Trying to Connect")
	serialPort.port = portName
	serialPort.baudrate = 9600
	serialPort.parity = 'N'
	serialPort.writeTimeout = 0
	
	serialPort.open()
	if(serialPort.isOpen()):
		print("Connected")
		return True
	
	return False

#Disconnects from Sparki
def disconnect():
	print("Disconnecting...")
	serialPort.close()
	#if(serialPort.isClosed()):
	#	print("Disconnected")

#Sends entire path to Sparki, first the length then each locations (x,y)
def sendPath(path, itemLoc):
	serialPort.write(pack('i', len(path)))
	print("Sending: ", len(path))
	while(not readOK()):
		pass
	for goal in path:
		sendLoc(goal[0], goal[1])
		if goal != itemLoc:
			serialPort.write(pack('i', 0))
		else:
			serialPort.write(pack('i', 1))
		while(not readOK()):
			pass

	print("Path Sent")

#Sends (x,y) location to Sparki
def sendLoc(x, y):
	print("Sending x:", x)
	serialPort.write(pack('f', x))
	print("Sending y:", y)
	serialPort.write(pack('f', y))

#Reads a 1 from Sparki saying its ok to send next location
def readOK():
	if(unpack('?',serialPort.read(size = 1))[0]):
		print("It's A OK")
		return True
	return False

#Builds a graph linking nodes that are at most 1 meter apart
def buildGraph():
	for i in range(len(landMarks)):
		neighbor = []
		for j in range(len(landMarks)):
			if i == j:
				continue
			if getDist(landMarks[i][0], landMarks[j][0], landMarks[i][1], landMarks[j][1]) <= 100:
				neighbor.append(j)
		graph.append(neighbor)

#Get closest landmark to given (x,y) coordinates
def getClosestLandmark(x, y):
	minDist = float("inf")
	minLandmark = 0
	for i in range(len(landMarks)):
		dist = getDist(landMarks[i][0], x, landMarks[i][1], y)
		if(dist < minDist):
			minDist = dist
			minLandmark = i
	return minLandmark

#Gets distance between two points
def getDist(x1, y1, x2 ,y2):
	return sqrt(pow(y2 - y1, 2) + pow(x2 - x1, 2))

#Creates path from the starting position to the ending position using BFS
def findPath(startX, startY, endX, endY):
	startLandmark = getClosestLandmark(startX, startY)
	lastLandmark = getClosestLandmark(endX, endY)
	start = [startX, startY]
	destination = [endX, endY]
	if start == destination:
		return [start]
	elif start == landMarks[startLandmark] == landMarks[lastLandmark] or landMarks[startLandmark] == landMarks[lastLandmark] == destination:
		return reversePath([start, destination])
	elif startLandmark == lastLandmark:
		return reversePath([start, landMarks[startLandmark], destination])
	Q = deque([startLandmark])
	parents = dict()
	parents[startLandmark] = None

	while len(Q) != 0:
		parent = Q.popleft()
		for neighbor in graph[parent]:
			if neighbor == lastLandmark:
				parents[neighbor] = parent
				return createPath(parents, lastLandmark, start, destination)
			if neighbor not in parents:
				parents[neighbor] = parent
				Q.append(neighbor)

	#add if path doesnt exist
	return createPath(parents, lastLandmark)

#Creates path to goal after running a BFS
def createPath(parents, lastLandmark, start, goal):
	path = deque()
	path.append(landMarks[lastLandmark])
	if landMarks[lastLandmark] != goal:
		path.append(goal)
	parent = parents[lastLandmark]
	while parent != None:
		path.appendleft(landMarks[parent])
		parent = parents[parent]
	path.appendleft(start)
	return reversePath(path)

#Takes a path and append the reverse path back to the starting position
def reversePath(path):
	path = list(path)
	path = path[1:]
	reversePath = list(path)
	reversePath.reverse()
	for goal in reversePath[1:]:
		path.append(goal)
	return path

if __name__ == "__main__":
	connect()
	#buildGraph()
	print("Graph:", graph)
	print("LandMarks:", landMarks)
	while(input("Y or N:") != "N"):
		startX = float(input("Start X:"))
		startY = float(input("Start Y:"))
		endX = float(input("End X:"))
		endY = float(input("End Y:"))
		path = findPath(startX, startY, endX, endY)
		path.append([startX, startY])
		print(path)
		sendPath(path, [endX, endY])
	input()
	disconnect()
