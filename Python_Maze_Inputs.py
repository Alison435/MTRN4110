# Test Script to send detected horizontal and vertical
# walls to Flood Fill algorithm on Arduino
# When PhaseB_CV_Planing.ino is loaded onto Arduino

import serial
import time

COM_MOUSE = "COM31" #change this to match your COM port (bluetooth port). 31 for Bluetooth

def main():

    #Maze as of 15/7/19 (red chair cell is [0,0])


    # Send vertical array
    action_vert = raw_input("Input vertical array\n") #replace by computer vision
    ser.write(action_vert + "\n")
    print("Sent vertical array\n")

    action_hor = raw_input("Input horizontal array\n") #replace by computer vision
    ser.write(action_hor + "\n")
    print("Sent horizontal array\n")


    time.sleep(0.5)


    #mouse direction
    action_dir = input("Please input 0 (north), 1 (east), 2 (south), 3 (west)\n")
    if (action_dir == 0):
        ser.write('0\n')
    elif (action_dir == 1):
        ser.write('1\n')
    elif (action_dir == 2):
        ser.write('2\n')
    elif( action_dir == 3):
        ser.write('3\n')

    #mouse position row
    action_pos = raw_input("Please input start row\n")
    ser.write(action_pos + "\n")

    #mouse position col
    action_pos = raw_input("Please input start col\n")
    ser.write(action_pos + "\n")

#----------Execute MAIN Here-----------

# Establish serial connection COM31 through Bluetooth
ser = serial.Serial(COM_MOUSE, 9600, timeout = 50)
time.sleep(2)
main()
