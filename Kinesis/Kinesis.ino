#include <Mouse.h>
#include <Wire.h>

#define leftFinger A0
#define midFinger A1
#define rightFinger A2

#define RXLED 17

//threshold for activating clicks and scroll
double leftThreshold, midThreshold;
int initialValueRight;

//gyro & accelerometer values
int ax, ay, az;
int gx, gy, gz;
int temp;

//mouse movement values
int vx, vy;

//address of MPU-6050
const int mpuAddr=0x68;

void getSensorValues(){
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(mpuAddr,14,true);  // request a total of 14 registers
  ax=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  ay=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  az=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  temp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gx=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gy=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gz=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

//getting threshold for left and middle flex sensors
double getSensorThreshold(int pin, double sensitivity){
  int t1 = millis();
  int minim = 1024;
  int maxim = 0;
  while(millis() - t1 < 2000){
    int val = analogRead(pin);
    if(val < minim)
      minim = val;
    if(val > maxim)
      maxim = val; 
  }
  return minim+ sensitivity*(maxim-minim);
}

void setup() {
  Serial.begin(9600);
  Mouse.begin();
  Wire.begin();
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x6B); //PWR_MGMT_1 register
  Wire.write(0); // wake up MPU-6050
  Wire.endTransmission(true);
  pinMode(RXLED, OUTPUT);
  //blink leds to signify beginning of calibration
  for(int i=0; i<8; i++){
    TXLED0;
    digitalWrite(RXLED, HIGH);
    delay(250);
    digitalWrite(RXLED, LOW);
    TXLED1;
    delay(250);
  }
  //calibrate sensors
  initialValueRight = analogRead(rightFinger);
  leftThreshold = getSensorThreshold(leftFinger, 0.7);
  midThreshold = getSensorThreshold(midFinger, 0.7);
}

void loop(){
  //get sensor values
  getSensorValues();
  int leftVal = analogRead(leftFinger);
  int midVal = analogRead(midFinger);
  int rightVal = analogRead(rightFinger);

  //check left click
  if(leftVal < leftThreshold)
    Mouse.press(MOUSE_LEFT);
  else if(Mouse.isPressed(MOUSE_LEFT))
    Mouse.release(MOUSE_LEFT);
  
  //check right click
  if(midVal < midThreshold)
    Mouse.press(MOUSE_RIGHT);
  else if(Mouse.isPressed(MOUSE_RIGHT))
    Mouse.release(MOUSE_RIGHT);
    
  //check scroll
  if(analogRead(rightFinger) > initialValueRight + 15)
    Mouse.move(0,0,1);
  else if(analogRead(rightFinger) < initialValueRight - 40)
    Mouse.move(0,0,-1);

  //move mouse
  vy = -(gx+300)/200;
  vx = (gz-100)/200;
  
  Mouse.move(-vx, vy);
  delay(20);
}
