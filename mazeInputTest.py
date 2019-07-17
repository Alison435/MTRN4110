# Test Script to send detected horizontal and vertical
# walls to Flood Fill algorithm on Arduino
# When PhaseB_CV_Planing.ino is loaded onto Arduino

import serial
import time

COM_MOUSE = "COM31" #change this to match your COM port (bluetooth port). 31 for Bluetooth

def main():

    print("Sending vertical array\n")
    #Maze as of 15/7/19 (red chair cell is [0,0])
    # Send vertical array
    ser.write("10100000011010000001101010010110101001111010001001\n")
    #print ser.readline() # Read the newest output from the Arduino
    print("Sent vertical array\n")

    time.sleep(0.5)

    print("Sending horizontal array\n")
    # Send horizontal array
    ser.write("111111111011111111101000010010100001101011100111111111\n")
    #print ser.readline() # Read the newest output from the Arduino
    print("Sent horizontal array\n")

    while (True):
        print(ser.readline())

#----------Execute MAIN Here-----------

# Establish serial connection COM31 through Bluetooth
ser = serial.Serial(COM_MOUSE, 9600, timeout = 50)
time.sleep(2)
main()
