from nanpy.arduinotree import ArduinoTree
from nanpy.i2c import I2C_Master
from nanpy.serialmanager import SerialManager
import logging
import math
from nanpy.wire import Wire
from time import sleep

from nanpy import ArduinoApi

connection = SerialManager()
a = ArduinoApi(connection=connection)
wire = Wire(connection=connection)
MPU = 0x68 
c = 0
b = a.millis()
print("hello")
print(str(b))
wire.begin()                    
wire.beginTransmission(MPU)     
wire.write(0x6B)               
wire.write(0x00)                  
wire.endTransmission(True)

'''
wire.beginTransmission(MPU)
wire.write(0x1C)
wire.write(0x10)
wire.endTransmission(True)
  
wire.beginTransmission(MPU)
wire.write(0x1B)
wire.write(0x10)
wire.endTransmission(True)
'''

def loopfunc():
  currentTime = 0  
  gyroAngleX = 0
  gyroAngleY = 0
  yaw = 0
  for i in range(0,200):
     wire.beginTransmission(MPU)
     wire.write(0x3B)
     wire.endTransmission(False)
     wire.requestFrom(MPU, 6, True)
     AccX = (wire.read() << 8 | wire.read()) / 16384.0
     AccY = (wire.read() << 8 | wire.read()) / 16384.0
     AccZ = (wire.read() << 8 | wire.read()) / 16384.0
     accAngleX = (math.atan(AccY / math.sqrt(math.pow(AccX, 2) + math.pow(AccZ, 2))) * 180 / math.pi) - 0.58
     accAngleY = (math.atan(-1 * AccX / math.sqrt(math.pow(AccY, 2) + math.pow(AccZ, 2))) * 180 / math.pi) + 1.58
     previousTime = currentTime
     currentTime = a.millis()
     elapsedTime = (currentTime - previousTime) / 1000
     wire.beginTransmission(MPU)
     wire.write(0x43)
     wire.endTransmission(False)
     wire.requestFrom(MPU, 6, True)
     GyroX = (wire.read() << 8 | wire.read()) / 131.0
     GyroY = (wire.read() << 8 | wire.read()) / 131.0
     GyroZ = (wire.read() << 8 | wire.read()) / 131.0
     GyroX = GyroX + 0.56
     GyroY = GyroY - 2
     GyroZ = GyroZ + 0.79
     gyroAngleX = gyroAngleX + GyroX * elapsedTime
     gyroAngleY = gyroAngleY + GyroY * elapsedTime
     yaw =  yaw + GyroZ * elapsedTime
     roll = 0.96 * gyroAngleX + 0.04 * accAngleX
     pitch = 0.96 * gyroAngleY + 0.04 * accAngleY

     print("roll"+str(i)+":"+str(roll)+"/"+"pitch"+str(pitch)+"/"+"yaw"+str(yaw))
     print(str(currentTime))


def calculate_IMU_error():
  c = 0
  AccErrorX = 0
  AccErrorY = 0
  while (c < 200):
    wire.beginTransmission(MPU)
    wire.write(0x3B)
    wire.endTransmission(False)
    wire.requestFrom(MPU, 6, True)
    AccX = (wire.read() << 8 | wire.read()) / 16384.0 
    AccY = (wire.read() << 8 | wire.read()) / 16384.0 
    AccZ = (wire.read() << 8 | wire.read()) / 16384.0 

    AccErrorX = AccErrorX + ((math.atan((AccY) / math.sqrt(math.pow((AccX), 2) + math.pow((AccZ), 2))) * 180 / math.pi))
    AccErrorY = AccErrorY + ((math.atan(-1 * (AccX) / math.sqrt(math.pow((AccY), 2) + math.pow((AccZ), 2))) * 180 / math.pi))
    c = c + 1

  AccErrorX = AccErrorX / 200
  AccErrorY = AccErrorY / 200
  c = 0
  GyroErrorX = 0
  GyroErrorY = 0
  GyroErrorZ = 0
  while (c < 200):
    wire.beginTransmission(MPU)
    wire.write(0x43)
    wire.endTransmission(False)
    wire.requestFrom(MPU, 6, True)
    GyroX = wire.read() << 8 | wire.read()
    GyroY = wire.read() << 8 | wire.read()
    GyroZ = wire.read() << 8 | wire.read()
    
    GyroErrorX = GyroErrorX + (GyroX / 131.0)
    GyroErrorY = GyroErrorY + (GyroY / 131.0)
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0)
    c = c + 1
  
  GyroErrorX = GyroErrorX / 200
  GyroErrorY = GyroErrorY / 200
  GyroErrorZ = GyroErrorZ / 200

  print("\nAccErrorX: ")
  print(AccErrorX)
  print("\nAccErrorY: ")
  print(AccErrorY)
  print("\nGyroErrorX: ")
  print(GyroErrorX)
  print("\nGyroErrorY: ")
  print(GyroErrorY)
  print("\nGyroErrorZ: ")
  print(GyroErrorZ)

calculate_IMU_error()
loopfunc()