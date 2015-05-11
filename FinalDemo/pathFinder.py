"""
	Path Finder
"""

from math import sqrt, atan2, pi, sin, cos
from time import sleep
from serial import Serial
from collections import deque
from struct import pack, unpack

graph = []
landMarks = [[70, 0], [400,150], [200, 200]]

portName = "/dev/cu.ArcBotics-DevB-1"
serialPort = Serial()

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

def disconnect():
	print("Disconnecting...")
	serialPort.close()
	#if(serialPort.isClosed()):
	#	print("Disconnected")

def sendPath(path):
	serialPort.write(pack('i', len(path)))
	print("Sending: ", len(path))
	while(not readOK()):
		pass
	for goal in path:
		sendLoc(goal[0], goal[1])
		while(not readOK()):
			pass
	print("Path Sent")

def sendLoc(x, y):
	print("Sending x:", x)
	serialPort.write(pack('f', x))
	print("Sending y:", y)
	serialPort.write(pack('f', y))

def readOK():
	if(unpack('?',serialPort.read(size = 1))[0]):
		print("It's A OK")
		return True
	return False

#def findCircleCenters():
	
def buildGraph():
	for i in range(len(landMarks)):
		neighbors = []
		for j in range(len(landMarks)):
			if(i != j):
				neighbors.append(j)
		graph.append(neighbors)

def getClosestLandmark(x, y):
	minDist = float("inf")
	minLandmark = 0
	for i in range(len(landMarks)):
		dist = sqrt(pow(y - landMarks[i][1], 2) + pow(x - landMarks[i][0], 2))
		if(dist < minDist):
			minDist = dist
			minLandmark = i
	return minLandmark

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

def reversePath(path):
	reversePath = list(path)
	reversePath.reverse()
	for goal in reversePath[1:]:
		path.append(goal)
	return path

if __name__ == "__main__":
	#connect()
	buildGraph()
	print("Graph:", graph)
	print("LandMarks:", landMarks)
	path = findPath(float(input("Start X:")), float(input("Start Y:")), float(input("Destination X:")), float(input("Destination Y:")))
	print(path)
	#sendPath(path)
	input()
	#disconnect()
