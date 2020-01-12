from nanpy import (ArduinoApi, SerialManager)
from time import sleep

connection = SerialManager()
a = ArduinoApi(connection=connection)

from nanpy import Servo
MIN_PULSE_LENGTH = 1000
MAX_PULSE_LENGTH = 2000
motA = Servo(4)
motA.attached()
motA.writeMicroseconds(0)
val = input("Enter your value: ") 

def min_throttle():
    print("Sending minimum throttle")
    motA.writeMicroseconds(MIN_PULSE_LENGTH)

def max_throttle():
    print("Sending maximum throttle")
    motA.writeMicroseconds(MAX_PULSE_LENGTH)

def test_run():
    print("Running test in 3 \n")
    sleep(1)
    print(" 2\n")
    sleep(1)
    print(" 1...\n")
    sleep(1)
    
    for x in range(1200,1300):
        print("Pulse length = ")
        print(x)
        print("\n")
        motA.writeMicroseconds(x)
        sleep(0.2)

    print("\nSTOP")
    motA.writeMicroseconds(MIN_PULSE_LENGTH)

def switchcase(data):
    if data == 1 :
        min_throttle()
    if(data==2):
        max_throttle()
    if(data==3): 
        test_run()

switchcase(int(val))


