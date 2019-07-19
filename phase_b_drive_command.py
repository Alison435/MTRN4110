# Test script to send commands over Bluetooth

import serial
import time

COM_MOUSE = "COM31"

def main():

    # Establish serial connection COM31 through Bluetooth
    ser = serial.Serial(COM_MOUSE, 9600, timeout = 50)

    # Wait for Arduino to be reset
    time.sleep(2)

    # User inputs 1,2,3,4 to activate Driving Test Functions

    while (1):

        action = input("Press 1,2,3 or 4\n")

        if (action == 1):

            # Send 1 to activate drive between two cells
            ser.write('1\n')

        elif (action == 2):

            # Send 2 to activate spot turn
            ser.write('2\n')
            time.sleep(0.5)

            rotDirection = raw_input("Rotation Direction?\n")

            if (rotDirection == '-90'): #works

                ser.write('-90\n') #desire turn to right (CW)

            elif (rotDirection == '90'):

                ser.write('90\n') #desire turn to left (CWW)

        elif (action == 3):

            # Send 3 to activate driving straight for 5 cells
            ser.write('3\n')

        elif( action == 4):

            # Send 4 to activate square wave
            ser.write('4\n')

        else:

            print("Invalid Input")

#----------Execute MAIN Here-----------

main()

if __name__ == "__main__":
    main()

