#include<Gyver433.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#define G433_SPEED 10000

MPU6050 mpu;
const int buffersize = 50;     // количество итераций калибровки
const int acel_deadzone = 6;  // точность калибровки акселерометра (по умолчанию 8)
const int gyro_deadzone = 4;   // точность калибровки гироскопа (по умолчанию 2)

int16_t ax, ay, az, gx, gy, gz;
int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

Gyver433_RX<2,40> rx;
uint8_t fifoBuffer[45];

float errYAW, errPITCH, errROLL;
float errYAWP, errPITCHP, errROLLP;
float errYAWI=0, errPITCHI=0, errROLLI=0;
float tarYAW, tarPITCH, tarROLL;
float forYAW, forPITCH, forROLL;
float LFW, RFC, LBC, RBW;
float P=0.2,I=0.005,D=0.1;
float thrust;
int stat=0;
int calibrations=0;

struct JoystickPack
{
    int x1,y1;
    int x2,y2;
    bool button1, button2;
};

void setup() {
  attachInterrupt(0, getJOY, CHANGE);
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  pinMode(3,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(9,OUTPUT);
}

JoystickPack joystick;
float ypr[3];
Quaternion q;
VectorFloat gravity;
uint32_t tmr=0;
uint32_t ENGtmr=0;
uint32_t tmrpr=0;
int on;

void getJOY()
{
  rx.tickISR();
}

void meansensors() {
  long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;
  while (i < (buffersize + 101)) 
  { 
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  //читаем сырую
    if (i > 100 && i <= (buffersize + 100)) 
    { 
      buff_ax = buff_ax + ax;
      buff_ay = buff_ay + ay;
      buff_az = buff_az + az;
      buff_gx = buff_gx + gx;
      buff_gy = buff_gy + gy;
      buff_gz = buff_gz + gz;
    }
    if (i == (buffersize + 100)) 
    {
      mean_ax = buff_ax / buffersize;
      mean_ay = buff_ay / buffersize;
      mean_az = buff_az / buffersize;
      mean_gx = buff_gx / buffersize;
      mean_gy = buff_gy / buffersize;
      mean_gz = buff_gz / buffersize;
    }
    i++;
    delay(2);
  }
}

void calibration() 
{
  ax_offset = -mean_ax / 8;
  ay_offset = -mean_ay / 8;
  az_offset = (16384 - mean_az) / 8;
  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;
  while (1) 
  {
    int ready = 0;
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);
    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);
    meansensors();
    Serial.println("...");
    if (abs(mean_ax) <= acel_deadzone) ready++;
    else ax_offset = ax_offset - mean_ax / acel_deadzone;
    if (abs(mean_ay) <= acel_deadzone) ready++;
    else ay_offset = ay_offset - mean_ay / acel_deadzone;
    if (abs(16384 - mean_az) <= acel_deadzone) ready++;
    else az_offset = az_offset + (16384 - mean_az) / acel_deadzone;
    if (abs(mean_gx) <= gyro_deadzone) ready++;
    else gx_offset = gx_offset - mean_gx / (gyro_deadzone + 1);
    if (abs(mean_gy) <= gyro_deadzone) ready++;
    else gy_offset = gy_offset - mean_gy / (gyro_deadzone + 1);
    if (abs(mean_gz) <= gyro_deadzone) ready++;
    else gz_offset = gz_offset - mean_gz / (gyro_deadzone + 1);
    if (ready == 6) break;
  }
}


void loop() 
{   

  if(calibrations==0)
  {
    meansensors();
    calibrations++;
    Serial.println("reading sensors");
    delay(2000);
  }
  if(calibrations==1)
  {
    calibration();
    calibrations++;
  }
  if(calibrations==2)
  {
    Serial.println("\nFINISHED!");
    Serial.print("\nSensor readings with offsets:\t");
    Serial.print(mean_ax);
    Serial.print("\t");
    Serial.print(mean_ay);
    Serial.print("\t");
    Serial.print(mean_az);
    Serial.print("\t");
    Serial.print(mean_gx);
    Serial.print("\t");
    Serial.print(mean_gy);
    Serial.print("\t");
    Serial.println(mean_gz);
    Serial.print("Your offsets:\t");
    Serial.print(ax_offset);
    Serial.print(", ");
    Serial.print(ay_offset);
    Serial.print(", ");
    Serial.print(az_offset);
    Serial.print(", ");
    Serial.print(gx_offset);
    Serial.print(", ");
    Serial.print(gy_offset);
    Serial.print(", ");
    Serial.println(gz_offset);
    Serial.println("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
    Serial.println("Check that your sensor readings are close to 0 0 16384 0 0 0");
    Serial.println("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)");
    calibrations++;
  }
  if(calibrations==3)
  {
    if(rx.gotData())
    { 
      JoystickPack buffer; 
      if(rx.readData(buffer)) rx.readData(joystick);
    }
    
    if (millis() - tmr >= 10) 
    {  
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
      {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        tmr = millis();  // сброс таймера
      }

    
      Serial.print("x1, y1 ");
      Serial.print(joystick.x1);
      Serial.print(" ");
      Serial.print(joystick.y1);
      Serial.println();
      Serial.print("x2, y2 ");
      Serial.print(joystick.x2);
      Serial.print(" ");
      Serial.print(joystick.y2);
      Serial.println();
      Serial.print(degrees(ypr[0])); // вокруг оси x
      Serial.print(',');
      Serial.print(degrees(ypr[1])); // вокруг оси z
      Serial.print(',');
      Serial.print(degrees(ypr[2])); // вокруг оси y
      Serial.println();
    

      tarYAW   = joystick.y2*180/100;
      tarPITCH = 0-(joystick.x2*30/100);
      tarROLL  = joystick.y1*30/100;
      thrust   = joystick.x1;
      
      if(thrust>100) thrust=100;
      if(thrust<0)    thrust=0;

      errYAW   = tarYAW   - degrees(ypr[0]);
      errPITCH = tarPITCH - degrees(ypr[1]);
      errROLL  = tarROLL  - degrees(ypr[2]);
  
      if(stat==0)
      {
        errYAWP   = errYAW;
        errPITCHP = errPITCH;
        errROLLP  = errPITCH;
        stat++; 
      }

      forYAW   = P * errYAW   + D * (errYAW-errYAWP)/0.010     + I * errYAWI;
      forPITCH = P * errPITCH + D * (errPITCH-errPITCHP)/0.010 + I * errPITCHI;
      forROLL  = P * errROLL  + D * (errROLL-errROLLP)/0.010   + I * errROLLI;

      LFW = thrust - forYAW + forPITCH + forROLL;
      RBW = thrust - forYAW - forPITCH - forROLL;
      RFC = thrust + forYAW + forPITCH - forROLL;
      LBC = thrust + forYAW - forPITCH + forROLL;

      if(LFW>100) LFW=100;
      if(LFW<0)   LFW=0;
      if(RBW>100) RBW=100;
      if(RBW<0)   RBW=0;
      if(RFC>100) RFC=100;
      if(RFC<0)   RFC=0;
      if(LBC>100) LBC=100;
      if(LBC<0)   LBC=0;

      errYAWP   = errYAW;
      errPITCHP = errPITCH;
      errROLLP  = errROLL;

      errYAWI   = errYAWI   + errYAW   * 0.010;
      errPITCHI = errPITCHI + errPITCH * 0.010;
      errROLLI  = errROLLI  + errROLL  * 0.010;
    
      Serial.println("\\ENGINES//");
      Serial.print(LFW);
      Serial.print(",");
      Serial.print(RBW);
      Serial.print(",");
      Serial.print(RFC);
      Serial.print(",");
      Serial.print(LBC);
      Serial.println();
    

    }
  

  if(millis()-ENGtmr > 10)
  {
    ENGtmr=millis();
  }

  if(millis()-ENGtmr < LFW/10)       //LFW
  {
    digitalWrite(3,HIGH);
  }
  else
  {
    digitalWrite(3,LOW);
  }

  if(millis()-ENGtmr < RFC/10)       //RFC
  {
    digitalWrite(5,HIGH);
  }
  else
  {
    digitalWrite(5,LOW);
  }

  if(millis()-ENGtmr < RBW/10)       //RBW
  {
    digitalWrite(6,HIGH);
    on=1;
  }
  else
  {
    digitalWrite(6,LOW);
    on=0;
  }

  if(millis()-ENGtmr < LBC/10)       //LBC
  {
    digitalWrite(9,HIGH);
  }
  else
  {
    digitalWrite(9,LOW);
  }
  }

}