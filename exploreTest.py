# Test script to send commands over Bluetooth

import serial
import time

COM_MOUSE = "COM31" #After pairing bluetooth HC06 module with computer
#check COM port

def main():

    # Establish serial connection COM31 through Bluetooth
    ser = serial.Serial(COM_MOUSE, 9600, timeout = 50)

    # Wait for Arduino to be reset
    time.sleep(2)

# User input for manual control
    while (1):

        action = input("Press 1 (straight), 2 (left), 3 (right)\n")

        if (action == 1):

            # Send 1 to make robot drive
            ser.write('s\n')

        elif (action == 2):

            # Send 2 to make robot left (and move to next cell)
            ser.write('l\n')

        elif( action == 3):

            # Send 3 to make robot turn right (and move to next cell)
            ser.write('r\n')

        else:

            print("Invalid Input")

#----------Execute MAIN Here-----------

main()

if __name__ == "__main__":
    main()

