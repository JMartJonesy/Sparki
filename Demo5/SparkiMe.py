import Sparki

if __name__ == "__main__":
	sparki = Sparki.Sparki("/dev/cu.ArcBotics-DevB")
	connected = sparki.connect()
	if(connected):
		while(True):
			print(sparki.lineSense())		
