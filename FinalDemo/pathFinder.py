"""
	Path Finder
"""

from math import sqrt, atan2, pi, sin, cos
from time import sleep
from serial import Serial
from collections import deque
from struct import pack, unpack

graph = []
landMarks = [[20, 0], [400,150], [200, 200]]

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
	#serialPort.write(pack('c', '*'))
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

def findPath(startLandmark, endX, endY):
	lastLandmark = getClosestLandmark(endX, endY)
	print("Finish:", lastLandmark)
	if startLandmark == lastLandmark:
		return [landMarks[startLandmark], [endX. endY]]
	Q = deque([startLandmark])
	parents = dict()
	parents[startLandmark] = None

	while len(Q) != 0:
		parent = Q.popleft()
		for neighbor in graph[parent]:
			if neighbor == lastLandmark:
				parents[neighbor] = parent
				return createPath(parents, lastLandmark, [endX, endY])
			if neighbor not in parents:
				parents[neighbor] = parent
				Q.append(neighbor)

	#add if path doesnt exist
	return createPath(parents, lastLandmark)

def createPath(parents, lastLandmark, goal):
	path = deque()
	path.append(landMarks[lastLandmark])
	path.append(goal)
	parent = parents[lastLandmark]
	while parent != None:
		path.appendleft(landMarks[parent])
		parent = parents[parent]
	return path

if __name__ == "__main__":
	connect()
	buildGraph()
	print("Graph:", graph)
	print("LandMarks:", landMarks)
	startLandmark = getClosestLandmark(0, 0) #move to this landmark then start path
	print("Start:", startLandmark)
	path = findPath(startLandmark, float(input("Input X:")), float(input("Input Y:")))
	print(path)
	sendPath(path)
	input()
	disconnect()
