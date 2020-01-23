from nanpy import (ArduinoApi, SerialManager)
from time import sleep
from nanpy import Servo
from nanpy.arduinotree import ArduinoTree
from nanpy.i2c import I2C_Master
import logging
import math
import numpy as np
from nanpy.wire import Wire

connection = SerialManager()
a = ArduinoApi(connection=connection)
wire = Wire(connection=connection)

MIN_PULSE_LENGTH = 1000
MAX_PULSE_LENGTH = 2000
motA = Servo(0)
motB = Servo(0)
motC = Servo(0)
motD = Servo(0)

motA.attached()
motB.attached()
motC.attached()
motD.attached()

motA.writeMicroseconds(0)
motB.writeMicroseconds(0)
motC.writeMicroseconds(0)
motD.writeMicroseconds(0)

sleep(1) # /*Give some delay, 7s, to have time to connect

def pid_loop(new_error_list):
     print("Error values :"+str(new_error_list))
     rad_to_deg = 180/math.pi
     # """
     # ////////////////////////////////////////////////////////////////////////////////////
     # //PID gain and limit settings
     # ////////////////////////////////////////////////////////////////////////////////////
     # """
     pid_p_gain_roll = 1.3              #Gain setting for the pitch and roll P-controller (default = 1.3).
     pid_i_gain_roll = 0.04              #Gain setting for the pitch and roll I-controller (default = 0.04).
     pid_d_gain_roll = 18.0              #Gain setting for the pitch and roll D-controller (default = 18.0).
     pid_max_roll = 400                 #Maximum output of the PID-controller (+/-).

     pid_p_gain_pitch = pid_p_gain_roll  # //Gain setting for the pitch P-controller.
     pid_i_gain_pitch = pid_i_gain_roll  #//Gain setting for the pitch I-controller.
     pid_d_gain_pitch = pid_d_gain_roll  #//Gain setting for the pitch D-controller.
     pid_max_pitch = pid_max_roll          #//Maximum output of the PID-controller (+/-).

     pid_p_gain_yaw = 4.0                #//Gain setting for the pitch P-controller (default = 4.0).
     pid_i_gain_yaw = 0.02               #//Gain setting for the pitch I-controller (default = 0.02).
     pid_d_gain_yaw = 0.0                #//Gain setting for the pitch D-controller (default = 0.0).
     pid_max_yaw = 400                    # //Maximum output of the PID-controller (+/-).
     pid_i_roll = 0
     pid_i_pitch = 0
     pid_i_yaw = 0
     # """
     # /////////////////PID CONSTANTS/////////////////
     # """
     kp=3.55
     ki=0.005
     kd=2.05
     # """
     # ///////////////////////////////////////////////
     # """
     throttle=1000       #//initial value of throttle to the motors
     desired_angle = 0   # //This is the angle in which we whant the

     wire.begin() #//begin the wire comunication
     wire.beginTransmission(0x68)
     wire.write(0x6B)               
     wire.write(0x00)                  
     wire.endTransmission(True)
     time = a.millis() #//Start counting time in milliseconds
          
     Acceleration_angle = [0,0]
     Gyro_angle = [0,0,0]
     Total_angle = [0,0,0]
     error = [0,0,0]
     previous_error = [0,0,0]
     i=0
     while True:
          
          #      """
          # /////////////////////////////I M U/////////////////////////////////////
          #      """
          timePrev = time  #// the previous time is stored before the actual time read
          time = a.millis()  #// actual time read
          elapsedTime = float((time - timePrev) / 1000) 
          #      """
          #   /*The timeStep is the time that elapsed since the previous loop. 
          #    * This is the value that we will use in the formulas as "elapsedTime" 
          #    * in seconds. We work in ms so we haveto divide the value by 1000 
          #    to obtain seconds*/

          #   /*Reed the values that the accelerometre gives.
          #    * We know that the slave adress for this IMU is 0x68 in
          #    * hexadecimal. For that in the RequestFrom and the 
          #    * begin functions we have to put this value.*/
          #      """
          start = a.millis()
          wire.beginTransmission(0x68)
          wire.write(0x3B) #//Ask for the 0x3B register- correspond to AcX
          wire.endTransmission(stop=False)
          wire.requestFrom(0x68,14,True) 
          wiretime =  a.millis()
          l = (float(float(wiretime)-float(start))/1000.0)
          #print(l)
          #      """
          #    /*We have asked for the 0x3B register. The IMU will send a brust of register.
          #     * The amount of register to read is specify in the requestFrom function.
          #     * In this case we request 6 registers. Each value of acceleration is made out of
          #     * two 8bits registers, low values and high values. For that we request the 6 of them  
          #     * and just make then sum of each pair. For that we shift to the left the high values 
          #     * register (<<) and make an or (|) operation to add the low values.*/
          #      """
          Acc_rawX=(np.int16(wire.read()<<8|wire.read()))  #//each value needs two registres
          Acc_rawY=(np.int16(wire.read()<<8|wire.read()))
          Acc_rawZ=(np.int16(wire.read()<<8|wire.read()))
          Temp =(np.int16(wire.read()<<8|wire.read()))
          Gyr_rawX=(np.int16(wire.read()<<8|wire.read()))  #//Once again we shif and sum
          Gyr_rawY=(np.int16(wire.read()<<8|wire.read()))
          Gyr_rawZ=(np.int16(wire.read()<<8|wire.read()))
          readvalues=a.millis()
          m=(float(float(readvalues)-float(wiretime))/1000.0)
          #print(m)
          #      """
          #     /*///This is the part where you need to calculate the angles using Euler equations///*/
          
          #     /* - Now, to obtain the values of acceleration in "g" units we first have to divide the raw   
          #      * values that we have just read by 16384.0 because that is the value that the MPU6050 
          #      * datasheet gives us.*/
          #     /* - Next we have to calculate the radian to degree value by dividing 180º by the PI number
          #     * which is 3.141592654 and store this value in the rad_to_deg variable. In order to not have
          #     * to calculate this value in each loop we have done that just once before the setup void.
          #     */

          #     /* Now we can apply the Euler formula. The atan will calculate the arctangent. The
          #      *  pow(a,b) will elevate the a value to the b power. And finnaly sqrt function
          #      *  will calculate the rooth square.*/
          #      """
          #/*---X---*/
          Acceleration_angle[0] = math.atan((Acc_rawY/16384.0)/math.sqrt(math.pow((Acc_rawX/16384.0),2) + math.pow((Acc_rawZ/16384.0),2)))*rad_to_deg
          Acceleration_angle[0] = Acceleration_angle[0] - new_error_list[0]         
          #/*---Y---*/
          Acceleration_angle[1] = math.atan(-1*(Acc_rawX/16384.0)/math.sqrt(math.pow((Acc_rawY/16384.0),2) + math.pow((Acc_rawZ/16384.0),2)))*rad_to_deg
          Acceleration_angle[1] =  Acceleration_angle[1] - new_error_list[1]
          #print("Acc:"+str(Acceleration_angle[0])+"/"+str(Acceleration_angle[1]))
          #      """
          #    /*Now we read the Gyro data in the same way as the Acc data. The adress for the
          #     * gyro data starts at 0x43. We can see this adresses if we look at the register map
          #     * of the MPU6050. In this case we request just 4 values. W don¡t want the gyro for 
          #     * the Z axis (YAW).*/
          #      """
          # wire.beginTransmission(0x68)
          # wire.write(0x43) #//Gyro data first adress
          # wire.endTransmission(stop=False)
          # wire.requestFrom(0x68,6,True) #//Just 4 registers
     
          # Gyr_rawX=(np.int16(wire.read()<<8|wire.read()))  #//Once again we shif and sum
          # Gyr_rawY=(np.int16(wire.read()<<8|wire.read()))
          # Gyr_rawZ=(np.int16(wire.read()<<8|wire.read()))
          
          #      """
          #    /*Now in order to obtain the gyro data in degrees/seconda we have to divide first
          #    the raw value by 131 because that's the value that the datasheet gives us*/
          #      """
          #/*---X---*/
          #sleep(0)
          elapsedTime = 0.01
          Gyro_angle[0] = (Gyr_rawX/131.0) *elapsedTime
          Gyro_angle[0] = Gyro_angle[0] - new_error_list[2]
          # /*---Y---*/
          Gyro_angle[1] = (Gyr_rawY/131.0) *elapsedTime
          Gyro_angle[1] = Gyro_angle[1] - new_error_list[3]
          #/*---Z---*/
          Gyro_angle[2] = (Gyr_rawZ/131.0) *elapsedTime
          Gyro_angle[2] =Gyro_angle[2] - new_error_list[4]
          #print("Gyro:"+str(Gyro_angle[0])+"/"+str(Gyro_angle[1])+"/"+str(Gyro_angle[2]))
          
          #      """  Complimentary Filter
          #    /*Now in order to obtain degrees we have to multiply the degree/seconds
          #    *value by the elapsedTime.*/
          #    /*Finnaly we can apply the final filter where we add the acceleration
          #    *part that afects the angles and ofcourse multiply by 0.98 */
          #      """
          #  /*---X axis angle---*/
          #print("gyro =",Gyr_rawX/131.0)
          Total_angle[0] = 0.96 *(Total_angle[0] +Gyro_angle[0]) + 0.04*Acceleration_angle[0]
          #/*---Y axis angle---*/
          Total_angle[1] = 0.96 *(Total_angle[1]+ Gyro_angle[1]) + 0.04*Acceleration_angle[1]
          #-----Z axis angle---
          Total_angle[2] = (Gyro_angle[2])
          i=i+1
          print(i)
          print("Angle:"+str(Total_angle[0])+"/"+str(Total_angle[1])+"/"+str(elapsedTime))
          # /*Now we have our angles in degree and values from -10º0 to 100º aprox*/
     

     
          #      """
          # /*///////////////////////////P I D///////////////////////////////////*/
          # /*Remember that for the balance we will use just one axis. I've choose the x angle
          # to implement the PID with. That means that the x axis of the IMU has to be paralel to
          # the balance*/

          # /*First calculate the error between the desired angle and 
          # *the real measured angle*/
          #"""
          for j in range (0,3):
               error[j] = Total_angle[j] - desired_angle
           #    print("Error"+str(j)+":"+str(error[j]))

          #      """
          # /*Next the proportional value of the PID is just a proportional constant
          # *multiplied by the error*/
          #      """

          #/*---------------------roll-------------------------*/
          pid_p_roll = pid_p_gain_roll*error[0]
          if(-3 <error[0] <3):
               pid_i_roll = pid_i_roll+(pid_i_gain_roll*error[0]) 
          pid_d_roll = pid_d_gain_roll*((error[0] - previous_error[0])/elapsedTime)

          pid_output_roll = pid_p_roll + pid_i_roll + pid_d_roll
          #/*---------------------pitch-------------------------*/
          pid_p_pitch = pid_p_gain_pitch*error[1]
          if(-3 <error[1] <3):
               pid_i_pitch = pid_i_pitch+(pid_i_gain_pitch*error[1])    
          pid_d_pitch = pid_d_gain_pitch*((error[1] - previous_error[1])/elapsedTime)

          pid_output_pitch = pid_p_pitch + pid_i_pitch + pid_d_pitch
          #/*---------------------yaw-------------------------*/
          pid_p_yaw = pid_p_gain_yaw*error[2]
          if(-3 <error[2] <3):
               pid_i_yaw = pid_i_yaw+(pid_i_gain_yaw*error[2])    
          pid_d_yaw = pid_d_gain_yaw*((error[2] - previous_error[2])/elapsedTime)

          pid_output_yaw = pid_p_yaw + pid_i_yaw + pid_d_yaw
          
          
          #print("PID:"+str(pid_output_roll)+"/"+str(pid_output_pitch)+"/"+str(pid_output_yaw))
          #      """
          # /*We know taht the min value of PWM signal is 1000us and the max is 2000. So that
          # tells us that the PID value can/s oscilate more than -1000 and 1000 because when we
          # have a value of 2000us the maximum value taht we could sybstract is 1000 and when
          # we have a value of 1000us for the PWM sihnal, the maximum value that we could add is 1000
          # to reach the maximum 2000us*/
          #      """

          PID1 = pid_output_pitch + pid_output_roll + pid_output_yaw 
          PID2 = pid_output_pitch - pid_output_roll - pid_output_yaw 
          PID3 = pid_output_yaw - pid_output_roll - pid_output_pitch
          PID4 = pid_output_roll - pid_output_pitch - pid_output_yaw

          if(PID1 < -400):
               PID1=-400
          if(PID1 > 400):
               PID1=400

          if(PID2 < -400):
               PID2=-400
          if(PID2 > 400):
               PID2=400

          if(PID3 < -400):
               PID3=-400
          if(PID3 > 400):
               PID3=400

          if(PID4 < -400):
               PID4=-400
          if(PID4 > 400):
               PID4=400
          #*Finally we calculate the PWM width. We sum the desired throttle and the PID value*/
          esc_1 = throttle + PID4       #//Calculate the pulse for esc 1 (front-right - CCW).
          esc_2 = throttle + PID1        #//Calculate the pulse for esc 2 (rear-right - CW).
          esc_3 = throttle + PID2       #//Calculate the pulse for esc 3 (rear-left - CCW).
          esc_4 = throttle + PID3       #//Calculate the pulse for esc 4 (front-left - CW).
          #      """
          # /*Once again we map the PWM values to be sure that we won't pass the min
          # and max values. Yes, we've already maped the PID values. But for example, for 
          # throttle value of 1300, if we sum the max PID value we would have 2300us and
          # that will mess up the ESC.*/
          # //Right
          #      """
          if(esc_1 < 1050):
               esc_1 = 1050
          if(esc_1 > 1400):
               esc_1 = 1400

          if(esc_2 < 1050):
               esc_2 = 1050
          if(esc_2 > 1400):
               esc_2 = 1400

          if(esc_3 < 1050):
               esc_3 = 1050
          if(esc_3 > 1400):
               esc_3 = 1400

          if(esc_4 < 1050):
               esc_4 = 1050
          if(esc_4 > 1400):
               esc_4 = 1400
 
          print("motor:"+str(esc_1)+"/"+str(esc_2)+"/"+str(esc_3)+"/"+str(esc_4))
          #/*Finnaly using the servo function we create the PWM pulses with the calculated width for each pulse*/
          for k in range(0,3):
               previous_error[k] = error[k] #//Remember to store the previous error.

def calibration_gyro():
  accAngleX = 0
  accAngleY = 0
  gyroAngleX = 0
  gyroAngleY = 0
  gyroAngleZ = 0
  AccErrorX = 0
  AccErrorY = 0 
  GyroErrorX = 0
  GyroErrorY = 0
  GyroErrorZ = 0
  MPU = 0x68
  wire.begin()                    
  wire.beginTransmission(MPU)     
  wire.write(0x6B)               
  wire.write(0x00)                  
  wire.endTransmission(True)
  currentTime = 0  
  for i in range(0,202):
     #  previousTime = currentTime
     #  currentTime = a.millis()
     #  elapsedTime = (currentTime - previousTime) / 1000
     wire.beginTransmission(MPU)
     wire.write(0x3B)
     wire.endTransmission(stop=False)
     wire.requestFrom(MPU, 6, True) # // Read 6 registers total, each axis value is stored in 2 registers
     # //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
     
     AccX = (np.int16(wire.read() << 8 | wire.read())) / 16384.0 
     AccY = (np.int16(wire.read() << 8 | wire.read())) / 16384.0
     AccZ = (np.int16(wire.read() << 8 | wire.read())) / 16384.0
     if(i>1):
          #// Calculating Roll and Pitch from the accelerometer data
          accAngleX = (math.atan(AccY / math.sqrt(math.pow(AccX, 2) + math.pow(AccZ, 2))) * 180 / math.pi)
          accAngleY = (math.atan(-1 * AccX / math.sqrt(math.pow(AccY, 2) + math.pow(AccZ, 2))) * 180 / math.pi)
     AccErrorX = AccErrorX + accAngleX 
     AccErrorY = AccErrorY + accAngleY
     
     #// === Read gyroscope data === //
     previousTime = currentTime
     currentTime = a.millis()
     elapsedTime = (currentTime - previousTime) / 1000
     wire.beginTransmission(MPU)
     wire.write(0x43) #// Gyro data first register address 0x43
     wire.endTransmission(stop=False)
     wire.requestFrom(MPU,6,True) #// Read 4 registers total, each axis value is stored in 2 registers
     GyroX = (np.int16(wire.read() << 8 | wire.read())) / 131.0 #// For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
     GyroY = (np.int16(wire.read() << 8 | wire.read())) / 131.0 
     GyroZ = (np.int16(wire.read() << 8 | wire.read())) / 131.0
     elapsedTime = 0.01
     #// Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
     if(i>1):
          gyroAngleX = GyroX * elapsedTime # // deg/s * s = deg
          gyroAngleY = GyroY * elapsedTime
          gyroAngleZ = GyroZ * elapsedTime
     GyroErrorX = GyroErrorX + gyroAngleX 
     GyroErrorY = GyroErrorY + gyroAngleY 
     GyroErrorZ = GyroErrorZ + gyroAngleZ
     
  AccErrorX = AccErrorX/200
  AccErrorY = AccErrorY/200 
  GyroErrorX = GyroErrorX/200
  GyroErrorY = GyroErrorY/200
  GyroErrorZ = GyroErrorZ/200
  error_list = [AccErrorX,AccErrorY,GyroErrorX,GyroErrorY,GyroErrorZ]
  print("sending error list :"+str(error_list))
  return (error_list)    

new_error_list = calibration_gyro()
pid_loop(new_error_list)
