import time
import serial

COM_MOUSE = "COM31"

def main():
    time.sleep(1)
    ser = serial.Serial(COM_MOUSE,9600,timeout = 50)

    f=open("bot.txt", "r")
    #contents =f.read() #read file data and store it in variable content

    f1 = f.readlines()

    for x in f1:
        print(x),
        ser.write(x)
        time.sleep(1.5)
    print("\n")


main()
