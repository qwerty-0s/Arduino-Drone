#include<Gyver433.h>
#define G433_SPEED 10000
Gyver433_TX<2> tx;
struct JoystickPack
{
    int x1,y1;
    int x2,y2;
    bool button1, button2;
};

void setup() 
{
    pinMode(14,INPUT); //x1
    pinMode(15,INPUT); //y1
    pinMode(16,INPUT); //x2
    pinMode(17,INPUT); //y2
    Serial.begin(9600);
}

    JoystickPack joystick;

void loop() 
{
  joystick.x1=map(analogRead(14),0,1023,-100,100);
  if(joystick.x1<4&&joystick.x1>0||joystick.x1>-4&&joystick.x1<0) joystick.x1=0;
  joystick.y1=map(analogRead(15),0,1023,-100,100);
  if(joystick.y1<4&&joystick.y1>0||joystick.y1>-4&&joystick.y1<0) joystick.y1=0;
  joystick.x2=map(analogRead(16),0,1023,-100,100);
  if(joystick.x2<4&&joystick.x2>0||joystick.x2>-4&&joystick.x2<0) joystick.x2=0;
  joystick.y2=map(analogRead(17),0,1023,-100,100);
  if(joystick.y2<4&&joystick.y2>0||joystick.y2>-4&&joystick.y2<0) joystick.y2=0;

  tx.sendData(joystick);
  delay(10);
}
