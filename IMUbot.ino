#include <math.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include "MPU6050_6Axis_MotionApps20.h"
#include "I2Cdev.h"

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

Quaternion q;         // [w, x, y, z]         quaternion container
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

double bot_angle;
double prev_angle = 0;
double angle = 0;
double target_angle = 0;


double time;
int flag = 1;
int flag2 = 1;

int wheels = 3;
int dir[3] = { 4, 12, 10 };  //insert direction pins
int pwm[3] = { 5, 9, 11 };                                    //insert pwm pins
bool directions[3] = { true, false, true };
float speed;
int vel[3];
int wheel_angles[3] = {0, 120, 240};
int x = 0, y = 0;


int button[3] = {3, 6, 7};

void setup() {
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
  while (!Serial)
    ;  // wait for Leonardo enumeration, others continue immediately

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  //time = millis();
  pinMode(A3, OUTPUT);
  digitalWrite(A3, HIGH);

  for (int i = 0; i < 3; i++)
    pinMode(button[i], INPUT_PULLUP);
    //attachInterrupt(digitalPinToInterrupt(3), plus45, FALLING);


    for(int i =0; i<wheels; i++)
    {
        pinMode(dir[i], OUTPUT);
        pinMode(pwm[i], OUTPUT);
    }
    
}

double initial_angle, holo_angle;
int flag3 = 1;

void imu() {
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    mpu.resetFIFO();

  else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
     Serial.print("ypr\t");
    Serial.println(ypr[0] * 180/M_PI);
     Serial.print(" ");
     Serial.print(ypr[1] * 180/M_PI);
     Serial.print(" ");
     Serial.println(ypr[2] * 180/M_PI);                  
#endif
    if (millis() - time > 15000) {
      if (flag3 == 1) {
        target_angle = ypr[0] * 180 / M_PI;
        flag3 = 0;
        initial_angle = target_angle;
      }
      bot_angle = ypr[0] * 180 / M_PI;
      holo_angle = bot_angle - initial_angle;
    }  //else Serial.println(millis());
  }
}
String input = "0";
long flag4 = 0, time1, set_delay, flag5=1;

void get_button_input() {
  int press = 0;
  for (int i = 0; i < 3; i++)
    if (digitalRead(button[i]) == 0)
      press = button[i]; 
        
    switch (press) {
      case 4: target_angle++; delay(100); break;
      case 5: target_angle--; delay(100); break;
    }
}

void get_input() {
  if (Serial.available() > 0) {
    input = Serial.readString();
    target_angle = input.toDouble();
    if (target_angle > 180) target_angle -= 360;
    else if (target_angle < -180) target_angle += 360;
  }

  double joyx = analogRead(A1);
  double joyy = analogRead(A0);

  x = (map(joyx, 0, 1023, 100, -100) - 6.5 * (100 - abs(x)) / 100)/2;
  y = (map(joyy, 0, 1023, 100, -100) - 2.5 * (100 - abs(y)) / 100)/2;
/*
  Serial.print(x);
  Serial.print("\t");
  Serial.print(y);
  Serial.print("\t");*/
}

float tans(double a, double b) {
  if(a==0 && b==0) return 0;
  if (a >= 0)
    return atan(b / a);
  else
    return M_PI + atan(b / a);
}

int imu_pwm;

void holonomic() {
  int dirs[wheels];
  float ang = tans(x, y) - holo_angle * M_PI / 180;
  Serial.print(ang * 180 / M_PI);
  Serial.print("\t");
  speed = sqrt(x * x + y * y);
  float vel_x = speed * cos(ang);
  float vel_y = speed * sin(ang);
  
  for (int i = 0; i < wheels; i++) {
    vel[i] = vel_x * cos(wheel_angles[i] * M_PI / 180) + vel_y * sin(wheel_angles[i] * M_PI / 180) + imu_pwm;
    // if (vel[i] < 0) {
    //   //vel[i] *= -1;
    //   dirs[i] = !directions[i];
    // }
    dirs[i] = abs(vel[i]) / vel[i];
    if (dirs[i] < 0) dirs[i] = 0;
  }

  for (int i = 0; i < wheels; i++) {
    digitalWrite(dir[i], dirs[i]);
    analogWrite(pwm[i], abs(vel[i]));

  /*  Serial.print("\t \t");
    Serial.print(dirs[i]);
    Serial.print("\t");
    Serial.print(vel[i]);*/
  }
}

void rotate(double error) {
  imu_pwm = map(error, -180, 180, 50, -50);
}

void loop() {
  if (!(!mpuInterrupt && fifoCount < packetSize))
    imu();

  if (flag2 == 1) {
    target_angle = bot_angle;
    flag2 = 0;
  }
  get_button_input();
  get_input();
  double error;
  error = target_angle - bot_angle;
  //if (error > 180) error = error - 360;

  /*Serial.print(bot_angle);
  Serial.print("\t");
  Serial.print(target_angle);
  Serial.print("\t");
  Serial.print(initial_angle);
  Serial.print("\t");
  Serial.print(error);
  Serial.print("\t");
  Serial.print(holo_angle);
  Serial.print("\t");
  Serial.print(imu_pwm);
  Serial.print("\t");*/

  rotate(error);
  holonomic();
  Serial.println("");
}
/*void plus45()
{
  target_angle -= 10;
} */