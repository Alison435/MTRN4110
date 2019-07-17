# Test script to send commands over Bluetooth
# TO be used with sensors_drive.ino

import serial
import time
import keyboard

COM_MOUSE = "COM31" #After pairing bluetooth HC06 module with computer
#check COM port

def main():

    # Establish serial connection COM31 through Bluetooth
    ser = serial.Serial(COM_MOUSE, 9600, timeout = 50)

    # Wait for Arduino to be reset
    time.sleep(2)

    # User input for manual control
    mazeprintDone = 0

    while (mazeprintDone == 0):

        print(ser.readline()) #this will only print when maze is found

        if (keyboard.is_pressed('e')):
               mazeprintDone = 1
               time.sleep(0.5)
               break


#----------Execute MAIN Here-----------

main()

if __name__ == "__main__":
    main()

