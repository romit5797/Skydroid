#include <Wire.h>
#include <Servo.h>


Servo right_prop1;
Servo left_prop1;
Servo right_prop2;
Servo left_prop2;
float pid_i_roll=0,pid_p_roll,pid_d_roll;
float pid_i_yaw=0,pid_p_yaw,pid_d_yaw;
float pid_i_pitch=0,pid_p_pitch,pid_d_pitch;

/*MPU-6050 gives you 16 bits data so you have to create some 16int constants
 * to store the data for accelerations and gyro*/
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll, angle_yaw; 
 
 float pid_p_gain_roll = 3;               //Gain setting for the pitch and roll P-controller (default = 1.3).
float pid_i_gain_roll = 0;              //Gain setting for the pitch and roll I-controller (default = 0.04).
float pid_d_gain_roll = 0;              //Gain setting for the pitch and roll D-controller (default = 18.0).
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-).

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-).

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller (default = 4.0).
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller (default = 0.02).
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller (default = 0.0).
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-).

int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
 
int PID1,PID2,PID3,PID4;
int esc_1,esc_2,esc_3,esc_4;
float Acceleration_angle[2]= {0,0};
float Gyro_angle[3]={0,0,0};
float Total_angle[3]={0,0,0};
int c =0;
float err[6]={0,0,0,0,0,0};
float AccErrorX=0, AccErrorY=0, AccErrorZ=0, GyroErrorX=0, GyroErrorY=0, GyroErrorZ=0;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
int16_t AccX, AccY, AccZ;
int16_t GyroX, GyroY, GyroZ;



float elapsedTime, time, timePrev;
float rad_to_deg = 180/3.141592654;

float PID, error_r,error_p,error_y, previous_error_r,previous_error_p,previous_error_y;
float pid_p=0;
float pid_i=0;
float pid_d=0;
/////////////////PID CONSTANTS/////////////////
double kp=3.55;//3.55
double ki=0.005;//0.003
double kd=2.05;//2.05
///////////////////////////////////////////////

double throttle=1060; //initial value of throttle to the motors
float desired_angle = 0; //This is the angle in which we whant the
                         //balance to stay steady
int g=0;
float ax,ay,az;


void setup() {
  Wire.begin(); //begin the wire comunication
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(115200);


  time = millis(); //Start counting time in milliseconds
  /*In order to start up the ESCs we have to send a min value
   * of PWM to them before connecting the battery. Otherwise
   * the ESCs won't start up or enter in the configure mode.
   * The min value is 1000us and max is 2000us, REMEMBER!*/
  left_prop1.writeMicroseconds(1000); 
  left_prop2.writeMicroseconds(1000); 
  right_prop1.writeMicroseconds(1000);
  right_prop2.writeMicroseconds(1000);
  Wire.beginTransmission(0x68);                   //Start communication with the MPU-6050.
  Wire.write(0x6B);                                            //We want to write to the GYRO_CONFIG register (1B hex).
  Wire.write(0x00);                                          //Set the register bits as 00001000 (500dps full scale).
  Wire.endTransmission();
  Serial.println("Starting..");
  calculate_IMU_error();
  Acceleration_angle[0]=0;
  
  
   
  Serial.println("done with imu calibration");
}

void loop() {

/////////////////////////////I M U/////////////////////////////////////
    timePrev = time;  // the previous time is stored before the actual time read
    time = millis();  // actual time read
    elapsedTime = (time - timePrev) / 1000; 
    elapsedTime = 0.003;
    g++;
  /*The tiemStep is the time that elapsed sincee previous loop. 
   * This is the value that we will use in the formulas as "elapsedTime" 
   * in seconds. We work in ms so we haveto divide the value by 1000 
   to obtain seconds*/

  /*Reed the values that the accelerometre gives.
   * We know that the slave adress for this IMU is 0x68 in
   * hexadecimal. For that in the RequestFrom and the 
   * begin functions we have to put this value.*/
     ;
    
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);
   /*We have asked for the 0x3B register. The IMU will send a brust of register.
    * The amount of register to read is specify in the requestFrom function.
    * In this case we request 6 registers. Each value of acceleration is made out of
    * two 8bits registers, low values and high values. For that we request the 6 of them  
    * and just make then sum of each pair. For that we shift to the left the high values 
    * register (<<) and make an or (|) operation to add the low values.*/
     
     Acc_rawX=Wire.read()<<8|Wire.read(); //each value needs two registres
     ax = Acc_rawX/16384.0;
     Acc_rawY=Wire.read()<<8|Wire.read();
     ay = Acc_rawY/16384.0;
     Acc_rawZ=Wire.read()<<8|Wire.read();
     az = Acc_rawZ/16384.0;
     
    
     
 
    /*///This is the part where you need to calculate the angles using Euler equations///*/
    
    /* - Now, to obtain the values of acceleration in "g" units we first have to divide the raw   
     * values that we have just read by 16384.0 because that is the value that the MPU6050 
     * datasheet gives us.*/
    /* - Next we have to calculate the radian to degree value by dividing 180º by the PI number
    * which is 3.141592654 and store this value in the rad_to_deg variable. In order to not have
    * to calculate this value in each loop we have done that just once before the setup void.
    */

    /* Now we can apply the Euler formula. The atan will calculate the arctangent. The
     *  pow(a,b) will elevate the a value to the b power. And finnaly sqrt function
     *  will calculate the rooth square.*/
     /*---X---*/
 /*---X---*/
     Acceleration_angle[0] = (atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg) - err[0];
    
     /*---Y---*/
     Acceleration_angle[1] = (atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg) - err[1];
     
 
   /*Now we read the Gyro data in the same way as the Acc data. The adress for the
    * gyro data starts at 0x43. We can see this adresses if we look at the register map
    * of the MPU6050. In this case we request just 4 values. W don¡t want the gyro for 
    * the Z axis (YAW).*/
    
   Wire.beginTransmission(0x68);
   Wire.write(0x43); //Gyro data first adress
   Wire.endTransmission(false);
   Wire.requestFrom(0x68,6,true); //Just 4 registers
   
   Gyr_rawX=Wire.read()<<8|Wire.read(); //Once again we shif and sum
   //Gyr_rawX = Gyr_rawX/131.-err[3] ;
  
   Gyr_rawY=Wire.read()<<8|Wire.read();
   //Gyr_rawY = Gyr_rawY-err[4] ;
   
   Gyr_rawZ=Wire.read()<<8|Wire.read();
   //Gyr_rawZ = Gyr_rawZ -err[5];
  
   /*Now in order to obtain the gyro data in degrees/seconda we have to divide first
   the raw value by 131 because that's the value that the datasheet gives us*/

   /*---X---*/
   Gyro_angle[0] = (Gyr_rawX/131.0) - err[3]; 
   /*---Y---*/
   Gyro_angle[1] = (Gyr_rawY/131.0) - err[4];
   Gyro_angle[2] = (Gyr_rawZ/131.0) - err[5];

   /*Now in order to obtain degrees we have to multiply the degree/seconds
   *value by the elapsedTime.*/
   /*Finnaly we can apply the final filter where we add the acceleration
   *part that afects the angles and ofcourse multiply by 0.98 */
   gyroAngleX =  Gyro_angle[0]*elapsedTime;
   gyroAngleY =  Gyro_angle[1]*elapsedTime;
   gyroAngleZ =  Gyro_angle[2]*elapsedTime;
  
   if(g>=0){  
   /*---X axis angle---*/
   Total_angle[0] = 0.98 *(Total_angle[0]+gyroAngleX) + 0.02*Acceleration_angle[0];
   /*---Y axis angle---*/
   Total_angle[1] = 0.98 *(Total_angle[1]+gyroAngleY) + 0.02*Acceleration_angle[1];
   Total_angle[2] = (Total_angle[2]+gyroAngleZ) ;

   
   /*Now we have our angles in degree and values from -10º0 to 100º aprox*/
    //Serial.println(Total_angle[1]);

   

/*///////////////////////////P I D///////////////////////////////////*/
/*First calculate the error between the desired angle and 
*the real measured angle*/
error_r = Total_angle[1]; 
Serial.println(":::");//- desired_angle
Serial.print(Total_angle[1]);//- desired_angle
Serial.print("/");//- desired_angle
Serial.print(Gyro_angle[1]);//- desired_angle
Serial.print("/");//- desired_angle
Serial.print(Acceleration_angle[1]);//-
error_p = 0;//Total_angle[1]*1.7 - desired_angle;
error_y = 0;//Total_angle[2]*1.7 - desired_angle;
    
/*Next the proportional value of the PID is just a proportional constant
*multiplied by the error*/
pid_p_roll = pid_p_gain_roll*error_r;
if(-3 <error_r <3){
     pid_i_roll = pid_i_roll+(pid_i_gain_roll*error_r) ;
}
pid_d_roll = pid_d_gain_roll*((error_r - previous_error_r));

pid_output_roll = pid_p_roll + pid_i_roll + pid_d_roll;
#/*---------------------pitch-------------------------*/
pid_p_pitch = pid_p_gain_pitch*error_p ;
if(-3 <error_p <3){
     pid_i_pitch = pid_i_pitch+(pid_i_gain_pitch*error_p) ;
}   
pid_d_pitch = pid_d_gain_pitch*((error_p - previous_error_p)/elapsedTime);

pid_output_pitch = pid_p_pitch + pid_i_pitch + pid_d_pitch;
#/*---------------------yaw-------------------------*/
pid_p_yaw = pid_p_gain_yaw*error_y;
if(-3 <error_y <3){
     pid_i_yaw = pid_i_yaw+(pid_i_gain_yaw*error_y)    ;
}
pid_d_yaw = pid_d_gain_yaw*((error_y - previous_error_y)/elapsedTime);

pid_output_yaw = pid_p_yaw + pid_i_yaw + pid_d_yaw;
          


/*The final PID values is the sum of each of this 3 parts*/
PID1 = pid_output_pitch + pid_output_roll + pid_output_yaw ;
PID2 = pid_output_pitch - pid_output_roll - pid_output_yaw ;
PID3 = pid_output_yaw - pid_output_roll - pid_output_pitch;
PID4 = pid_output_roll - pid_output_pitch - pid_output_yaw;

if(PID1 < -400){
     PID1=-400;
}
if(PID1 > 400){
     PID1=400;
}

if(PID2 < -400){
     PID2=-400;
}
if(PID2 > 400){
     PID2=400;
}

if(PID3 < -400){
     PID3=-400;
}
if(PID3 > 400){
     PID3=400;
}

if(PID4 < -400){
     PID4=-400;
}
if(PID4 > 400){
     PID4=400;
}

/*We know taht the min value of PWM signal is 1000us and the max is 2000. So that
tells us that the PID value can/s oscilate more than -1000 and 1000 because when we
have a value of 2000us the maximum value taht we could sybstract is 1000 and when
we have a value of 1000us for the PWM sihnal, the maximum value that we could add is 1000
to reach the maximum 2000us*/

/*Finally we calculate the PWM width. We sum the desired throttle and the PID value*/
esc_1 = throttle + PID4;       //Calculate the pulse for esc 1 (front-right - CCW).
esc_2 = throttle + PID1 ;       //Calculate the pulse for esc 2 (rear-right - CW).
esc_3 = 1060;//throttle + PID2  ;     //Calculate the pulse for esc 3 (rear-left - CCW).
esc_4 = 1060;//throttle + PID3 ;    //Calculate the pulse for esc 4 (front-left - CW).

if(esc_1 < 1060){
     esc_1 = 1060;
}
if(esc_1 > 1300){
     esc_1 = 1300;
}

if(esc_2 < 1060){
     esc_2 = 1060;
}
if(esc_2 > 1300){
     esc_2 = 1300;
}

if(esc_3 < 1060){
     esc_3 = 1060;
}
if(esc_3 > 1300){
     esc_3 = 1300;
}
if(esc_4 < 1060){
     esc_4 = 1060;
}
if(esc_4 > 1300){
     esc_4 = 1300;
}

/*Finnaly using the servo function we create the PWM pulses with the calculated
width for each pulse*/
left_prop1.writeMicroseconds(esc_4);
left_prop2.writeMicroseconds(esc_3);
right_prop1.writeMicroseconds(esc_1);
right_prop2.writeMicroseconds(esc_2);

previous_error_r = error_r; //Remember to store the previous error.
previous_error_p = error_p; //Remember to store the previous error.
previous_error_y = error_y; //Remember to store the previous error.


 }

}


void calculate_IMU_error() {
 
  while (c < 2000) {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);
     
    AccX = (Wire.read() << 8 | Wire.read()) ;
    AccY = (Wire.read() << 8 | Wire.read()) ;
    AccZ = (Wire.read() << 8 | Wire.read()); 
    // Sum all readings
 
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    
  
 

  // Read gyro values 200 times
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);
   
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();

    GyroErrorX = GyroErrorX + GyroX/131.0;
    GyroErrorY = GyroErrorY + GyroY/131.0;
    GyroErrorZ = GyroErrorZ + GyroZ/131.0;
    
    c++;

  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 2000;
  AccErrorY = AccErrorY / 2000;
  AccErrorZ = 0;
  
  GyroErrorX = GyroErrorX / 2000;
  GyroErrorY = GyroErrorY / 2000;
  GyroErrorZ = GyroErrorZ / 2000;
  // Print the error values on the Serial Monitor
  err[0]= AccErrorX;
  err[1] = AccErrorY;
  err[2] = AccErrorZ;
  err[3] = GyroErrorX;
  err[4] = GyroErrorY;
  err[5] = GyroErrorZ;

  for(int x=0;x<6;x++){
  Serial.print(err[x]);
Serial.print("/");}
  
}
 
