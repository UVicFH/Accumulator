#this library is what lets the script connect to the arduino serial terminal 
import serial
from xlwt import Workbook

def main():

	print('starting program now')
	#telling script which port to use and baud#
	ser = serial.Serial('COM6', 115200 )
	line = []
	#excel module stuff
	wb = Workbook()
	sheet = wb.add_sheet('accumulator sheet')

	index_row = 1
	while True:
                #decoding byte type to string
                line_not_decoded = ser.readline().decode('utf-8')
                #line split up by ',' into a list
                line = line_not_decoded.split(",")

                index_col = 1
                for word in line:
                        
                        sheet.write(index_row, index_col, word)
                        index_col = index_col + 1
                index_row = index_row + 1
                #name of excel sheet
                wb.save('accumulator_sheet.xls')
                
if __name__ == "__main__":
	main()
