# Test script to send array of commands for robot to use
# Testing Phase B Vision Part D

import serial
import time

COM_MOUSE = "COM31"

def main():

    # Establish serial connection COM31 through Bluetooth
    ser = serial.Serial(COM_MOUSE, 9600, timeout = 50)

    time.sleep(2)

    # Put robot into auto mode
    ser.write('a\n')

    time.sleep(2)

    # Send command string to Micromouse
    ser.write("^^>^^>\n")

#----------Execute MAIN Here-----------

main()

if __name__ == "__main__":
    main()

