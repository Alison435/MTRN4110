# Test Script to send detected horizontal and vertical
# walls to Flood Fill algorithm on Arduino

import serial
import time

COM_MOUSE = "COM7" #change this to match your COM port (bluetooth port)

def main():

    print("Sending vertical array\n")
    # Send vertical array
    ser.write("10001000011110010111100122111110001222211010001011\n")
    print ser.readline() # Read the newest output from the Arduino

    time.sleep(0.5)

    print("Sending horizontal array\n")
    # Send horizontal array
    ser.write("111111111001101122010111221101101222010101010111111111\n")
    print ser.readline() # Read the newest output from the Arduino

    # Prints the results of the inputted arrays
    print ser.readline()
    print ser.readline()
    print ser.readline()
    print ser.readline()
    print ser.readline()
    print ser.readline() # Should be COMMANDS FOR ROBOT MOVEMENT:...

#----------Execute MAIN Here-----------

# Establish serial connection COM31 through Bluetooth
ser = serial.Serial(COM_MOUSE, 9600, timeout = 50)
time.sleep(2)
main()
