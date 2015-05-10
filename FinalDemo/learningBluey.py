from serial import Serial
from struct import pack, unpack

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
#	if(serialPort.isClosed()):
#		print("Disconnected")

if __name__ == "__main__":
	connect()
	serialPort.write(pack('<f', 100.0))
	while(True):
		pass
	disconnect()
