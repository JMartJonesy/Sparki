"""
	Path Finder
"""

from math import sqrt, atan2, pi, sin, cos
from time import sleep
from serial import Serial
from collections import deque
from struct import pack, unpack

graph = []
landMarks = [[20, 0], [40,20], [20, 20]]

portName = "/dev/cu.ArcBotics-DevB"
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
	if(serialPort.isClose()):
		print("Disconnected")

def sendPath(path):
	for i in path:
		serialPort.write('!B', '!')
		sendLoc(landMarks[i][0], landMarks[i][1])
		if(not readOK()):
			print("Shits Fucked")
			break
	print("Path Sent")
	serialPort.write('!B', '*')

def sendLoc(x, y):
	print("Sending x:", x)
	serialPort.write(pack('f', x))
	readOK()
	print("Sending y:", y)
	serialPort.write(pack('f', y))
	readOK()

def readOK():
	if(str(serialPort.read(size = 1)) == "K"):
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
	Q = deque([startLandmark])
	parents = dict()
	parents[startLandmark] = None

	while len(Q) != 0 and Q[0] != lastLandmark:
		parent = Q.popleft()
		for neighbor in graph[parent]:
			if neighbor == lastLandmark:
				parents[neighbor] = parent
				return createPath(parents, lastLandmark)
			if neighbor not in parents:
				parents[neighbor] = parent
				Q.append(neighbor)

	return createPath(parents, lastLandmark)

def createPath(parents, lastLandmark):
	path = deque()
	if parents[lastLandmark] != None:
		path.append(lastLandmark)
		parent = parents[lastLandmark]
		while parent != None:
			path.appendleft(parent)
			parent = parents[parent]
	return path

if __name__ == "__main__":
	buildGraph()
	print("Graph:", graph)
	print("LandMarks:", landMarks)
	startLandmark = getClosestLandmark(0, 0) #move to this landmark then start path
	print("Start:", startLandmark)
	path = findPath(startLandmark, float(input("Input X:")), float(input("Input Y:")))
	print(path)
	#connect()
	#while(True):
	#	pass
	phi = atan2(0 - 0, 20 - 0);
	print((atan2(sin(phi - 0), cos(phi - 0))) * 180 / pi)
	phi = atan2(200 - 0, 400 - 20);
	print((atan2(sin(phi - pi / 2), cos(phi - pi / 2))) * 180 / pi)
	phi = atan2(20 - 200, 200 - 400);
	print((atan2(sin(phi - pi / 2), cos(phi - pi / 2))) * 180 / pi)
