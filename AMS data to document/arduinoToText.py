#this library is what lets the script connect to the arduino serial terminal 
import serial


def main():

	print('starting program now')
	#telling script which port to use and baud#
	ser = serial.Serial('COM4', 115200 )
	
	while True:
		print (ser.readline())

if __name__ == "__main__":
	main()