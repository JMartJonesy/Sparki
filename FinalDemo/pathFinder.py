"""
	Path Finder
"""

from math import sqrt, atan2, pi, sin, cos
from time import sleep
from serial import Serial
from collections import deque
from struct import pack, unpack

graph = []
#graph = [[1, 4], [0, 4], [3],[2], [0, 1]]
landMarks = [[64.45, -76.26], [21.33, -166.46], [77.77, -62.78], [107.35, -158.22], [70,0]]

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

def buildGraph():
	for i in range(len(landMarks)):
		neighbor = []
		for j in range(len(landMarks)):
			if i == j:
				continue
			if getDist(landMarks[i][0], landMarks[j][0], landMarks[i][1], landMarks[j][1]) <= 100:
				neighbor.append(j)
		graph.append(neighbor)

def getClosestLandmark(x, y):
	minDist = float("inf")
	minLandmark = 0
	for i in range(len(landMarks)):
		dist = getDist(landMarks[i][0], x, landMarks[i][1], y)
		if(dist < minDist):
			minDist = dist
			minLandmark = i
	return minLandmark

def getDist(x1, y1, x2 ,y2):
	return sqrt(pow(y2 - y1, 2) + pow(x2 - x1, 2))

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
	path = list(path)
	path = path[1:]
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
	while(input("Y or N:") != "N"):
		startX = float(input("Start X:"))
		startY = float(input("Start Y:"))
		endX = float(input("End X:"))
		endY = float(input("End Y:"))
		path = findPath(startX, startY, endX, endY)
		path.append([startX, startY])
		print(path)
		#sendPath(path, [endX, endY])
	input()
	#disconnect()
