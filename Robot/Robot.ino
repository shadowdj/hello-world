// SainSmart Instabots Upright Rover rev. 3.0
// Updatas at http://www.sainsmart.com

#include <Wire.h>
#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
#include <I2Cdev.h>
#include <MPU6050.h>

MPU6050 accelgyro;
MPU6050 initialize;
int16_t ax, ay, az;
int16_t gx, gy, gz;

#define Gry_offset 0  //The offset of the gyro
#define Gyr_Gain 131
#define Angle_offset 0  // The offset of the accelerator
#define RMotor_offset 0  // The offset of the Motor
#define LMotor_offset 0  // The offset of the Motor
#define pi 3.14159

float Angle_Delta, Angle_Recursive, Angle_Confidence;

float kp, ki, kd;
float Angle_Raw, Angle_Filtered, omega, dt;
float Angle_Vertical = 2.0;
float Turn_Speed = 0, Run_Speed = 0;
float LOutput, ROutput, Input, Output;

unsigned long preTime, lastTime;
float errSum, dErr, error, lastErr;
int timeChange;

long volatile Sum_Right, Sum_Right_Temp = 150, Sum_Left, Sum_Left_Temp = 150, Distance, Distance_Right, Distance_Left, Speed;


int TN1 = 23;
int TN2 = 22;
int ENA = 5;
int TN3 = 24;
int TN4 = 25;
int ENB = 4;


// ------------------------------------
// Data structure - Controller to Robot
// ------------------------------------
struct IncomingData  // Data from remote control
{
  int axis_1;
  int axis_2;
  int axis_3;
  int axis_4;
  int axis_5;
  int axis_6;
  int axis_7;
  int axis_8;
  int spare1;
  int spare2;
  int spare3;
  int spare4;
};

// Instantiate the structure

IncomingData incoming;

// ------------------------------------
// Data structure - Robot to Controller
// ------------------------------------

struct OutgoingData  // Datas send back to remote control
{
  float angle;
  float omega;
  int speed;
  float P;
  float I;
  float D;
  uint16_t null_1;
  uint16_t null_2;
};

// Instantiate the structure

OutgoingData outgoing;


void setup()
{
  Serial.begin(115200);
  Wire.begin();

  TCCR3A = _BV(COM3A1) | _BV(WGM31) | _BV(WGM30); // TIMER_3 @1K Hz, fast pwm
  TCCR3B = _BV(CS31);
  TCCR0A = _BV(COM0B1) | _BV(WGM01) | _BV(WGM00); // TIMER_0 @1K Hz, fast pwm
  TCCR0B = _BV(CS01) | _BV(CS00);

  /* If the robot was turned on with the angle over 45(-45) degrees,the wheels
   will not spin until the robot is in right position. */
  accelgyro.initialize();
  for (int i = 0; i < 200; i++) // Looping 200 times to get the real gesture when starting
  {
    Filter();
  }
  if (abs(Angle_Filtered) < 45)  // Start to work after cleaning data
  {
    omega = Angle_Raw = Angle_Filtered = 0;
    Output = error = errSum = dErr = 0;
    Filter();
    myPID();
  }
    
  // Left wheel H-Bridge
  
  pinMode(TN1, OUTPUT);
  pinMode(TN2, OUTPUT);
  
  // Right wheel H-Bridge
  
  pinMode(TN3, OUTPUT);
  pinMode(TN4, OUTPUT);
  
  // Right wheel PWM
  
  pinMode(ENA, OUTPUT);
  
  // Left wheel PWM
  
  pinMode(ENB, OUTPUT);
  
  pinMode(18, INPUT);
  pinMode(2, INPUT);
  pinMode(19, INPUT);
  pinMode(3, INPUT);
  

  attachInterrupt(4, State_A, FALLING);
  attachInterrupt(1, State_B, FALLING);

  // 24L01 initialization
  Mirf.cePin = 53;
  Mirf.csnPin = 48;
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  Mirf.setRADDR((byte *)"serv1");
  Mirf.payload = 24;
  Mirf.config();
  
    
}

void loop()
{
  // Test Program 
  for (int i = 0; i < 255; i += 10)
  {
    LOutput = i;
    ROutput = i;
    PWMControl();
    delay (1000);
    Sum_Left = 0;
    Sum_Right = 0;
    delay (1000);
    Serial.print("Speed = ");
    Serial.print(i);
    Serial.print(" Count = ");
    Serial.print(Sum_Right);
    Serial.print(" Count = ");
    Serial.println(Sum_Left);
  }
  
  while (1)
  {
    Receive();
    if ((micros() - lastTime) > 10000)
    {
      Filter();
      // If angle > 45 or < -45 then stop the robot
      if (abs(Angle_Filtered) < 45)
      {
        myPID();
        PWMControl();
      }
      else
      {
        digitalWrite(TN1, HIGH);
        digitalWrite(TN2, HIGH);
        digitalWrite(TN3, HIGH);
        digitalWrite(TN4, HIGH);
      }
      lastTime = micros();
    }
  }
}

void Receive()
{
  if (!Mirf.isSending() && Mirf.dataReady())
  {
    // Read datas from the romote controller
    Mirf.getData((byte *) &incoming);
    /*Serial.print("axis_1=");
    Serial.print(axis_x.axis_1);
    Serial.print("  axis_2=");
    Serial.print(axis_x.axis_2);
    Serial.print("  axis_3=");
    Serial.print(axis_x.axis_3);
    Serial.print("  axis_4=");
    Serial.print(axis_x.axis_4);
    Serial.print("  axis_5=");
    Serial.print(axis_x.axis_5);
    Serial.print("  axis_6=");
    Serial.print(axis_x.axis_6);
    Serial.print("  axis_7=");
    Serial.print(axis_x.axis_7);
    Serial.print("  axis_8=");
    Serial.println(axis_x.axis_8);*/

    Mirf.setTADDR((byte *)"clie1");
    Mirf.send((byte *) &outgoing);  // Send datas back to the controller

    
    //_______________________________________________________
    //  Axis 2 is right joystick X axis (Left/Right)
    //
    //  Else map input to -120 to +120
    
    Turn_Speed = map(incoming.axis_2, -530, 530, -120, 120);
    /*
    if (incoming.axis_2 >= 520) 
    {
      Turn_Speed = map(incoming.axis_2, 520, 1023, 0, 120);
    }
    else if (incoming.axis_2 <= 480)
    {
      Turn_Speed = map(incoming.axis_2, 480 , 0, 0, -120);
    }
    else
    {
      Turn_Speed = 0;
    }
    */

    //_______________________________________________________
    //  Axis 1 is right joystick Y axis (Forward/Back)
    //
    //  If abs(value) < 20 set to 0
    //  Else map input to -100 to +100

    Run_Speed = map(incoming.axis_1, -530, 530, -100, 100);
    /*
    if (incoming.axis_1 >= 520) 
    {
      Run_Speed = map(incoming.axis_1, 520, 1023, 0, 100);
    }
    else if (incoming.axis_1 <= 480)
    {
      Run_Speed = map(incoming.axis_1, 480, 0, 0, -100);
    }
    else
    {
      Run_Speed = 0;
    }
    
  */
  }
  else
  {
    incoming.axis_1 = incoming.axis_2 = 0;
  }
  

  
  // Read the analog potentiometers and convert the values to kp, ki, kd
  
  kp = float(analogRead(A0))/10.0;
  ki = float(analogRead(A1))/100.0;
  kd = float(analogRead(A2))/100.0;
  
  // Put the structure values into the structure (data)
  //   for transmission back to the controller
  
  outgoing.omega = omega;
  outgoing.angle = Angle_Filtered;
  outgoing.speed = Output;
  outgoing.P = kp; 
  outgoing.I = Run_Speed;
  outgoing.D = kd;
}

void Filter()
{
  // Raw datas
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Angle_Raw = (atan2(ay, az) * 180 / pi + Angle_offset);
  Angle_Raw = Angle_Raw - Angle_Vertical;
  omega = gx / Gyr_Gain + Gry_offset;
  // Filter datas to get the real gesture
  unsigned long now = micros();
  timeChange = now - preTime;
  preTime = now;
  dt = timeChange * 0.000001;
  Angle_Delta = (Angle_Raw - Angle_Filtered) * 0.64;
  Angle_Recursive = Angle_Delta * dt + Angle_Recursive;
  Angle_Confidence = Angle_Recursive + (Angle_Raw - Angle_Filtered) * 1.6 + omega;
  Angle_Filtered = Angle_Confidence * dt + Angle_Filtered;
}

void myPID()
{
//  kp = 22.000; 
//  ki = 0;
//  kd = 1.60;
  // Calculating the output values using the gesture values and the PID values.
  error = Angle_Filtered;
  errSum += error;
  dErr = error - lastErr;
  Output = kp * error + ki * errSum + kd * omega;
  lastErr = error;
  noInterrupts();
  if(abs(Sum_Left - Sum_Left_Temp) > 300)
  {
    Sum_Left = Sum_Left_Temp;
  }
  if(abs(Sum_Right - Sum_Right_Temp) > 300)
  {
    Sum_Right = Sum_Right_Temp;
  }
  Speed = (Sum_Right + Sum_Left) / 2;
  Distance += Speed + Run_Speed;
  Distance = constrain(Distance, -8000, 8000);
  Output += Speed * 2.4 + Distance * 0.025;
  Sum_Right_Temp = Sum_Right;
  Sum_Left_Temp = Sum_Right;
  Sum_Right = 0;
  Sum_Left = 0;

  ROutput = Output + Turn_Speed;
  LOutput = Output - Turn_Speed;
  interrupts();
}

void PWMControl()
{
  if (LOutput > 0)
  {
    digitalWrite(TN1, HIGH);
    digitalWrite(TN2, LOW);
  }
  else if (LOutput < 0)
  {
    digitalWrite(TN1, LOW);
    digitalWrite(TN2, HIGH);
  }
  else
  {
    OCR3A = 0;
  }
  if (ROutput > 0)
  {
    digitalWrite(TN3, HIGH);
    digitalWrite(TN4, LOW);
  }
  else if (ROutput < 0)
  {
    digitalWrite(TN3, LOW);
    digitalWrite(TN4, HIGH);
  }
  else
  {
    OCR0B = 0;
  }
  OCR3A = min(1023, (abs(LOutput * 4) + LMotor_offset * 4)); // Timer/Counter3 is a general purpose 16-bit Timer/Counter module
  OCR0B = min(255, (abs(ROutput) + RMotor_offset)); // Timer/Counter0 is a general purpose 8-bit Timer/Counter module
}


void State_A()
{
  if (digitalRead(18))
  {
    Sum_Right ++;
  }
  else
  {
    Sum_Right --;
  }
}

void State_B()
{
  if (!digitalRead(2))
  {
    Sum_Left ++;
  }
  else
  {
    Sum_Left --;
  }
}

