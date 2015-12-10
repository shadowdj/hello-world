// SainSmart Instabots Upright Rover rev. 3.0
// Updatas at http://www.sainsmart.com

#include <Wire.h>
#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
#include <I2Cdev.h>
#include <MPU6050.h>                  // IMU

//   Set up and initialize the MPU6050 (accelerometer and gyros)

MPU6050 accelgyro;
MPU6050 initialize;
int16_t ax, ay, az;        // X, Y, Z accelerometer values (position)
int16_t gx, gy, gz;        // X, Y, Z gyro values          (rate of change)

#define Gry_offset 0    // Offset of the gyro
#define Gyr_Gain 131    // Gain of the gyro (131 counts per 1 degree per second)
#define Angle_offset 0  // Offset of the accelerator


#define RMotor_offset 0  // The offset of the Motor
#define LMotor_offset 0  // The offset of the Motor
#define pi 3.14159

float Angle_Delta, Angle_Recursive, Angle_Confidence;

float kp, ki, kd;
float Angle_Raw, Angle_Filtered, omega, dt;
float Turn_Speed = 0, Run_Speed = 0;
float LOutput, ROutput, Input, Output;
float PValue, IValue, DValue;

unsigned long preTime, lastTime;
float errSum, dErr, error, lastErr;
int timeChange;

long Sum_Right, Sum_Right_Temp = 150, Sum_Left, Sum_Left_Temp = 150, Distance, Distance_Right, Distance_Left, Speed;

int TN1 = 23;    // Left wheel H-Bridge
int TN2 = 22;    // Left wheel H-Bridge
int ENA = 5;
int TN3 = 24;    // Right wheel H-Bridge
int TN4 = 25;    // Right wheel H-Bridge
int ENB = 4;

// ------------------------------------
// Data structure - Controller to Robot
// ------------------------------------
struct IncomingData  // Data from remote control
{
  uint16_t axis_1;
  uint16_t axis_2;
  uint16_t axis_3;
  uint16_t axis_4;
  uint16_t axis_5;
  uint16_t axis_6;
  uint16_t axis_7;
  uint16_t axis_8;
};

IncomingData incoming;

// ------------------------------------
// Data structure - Robot to Controller

struct OutgoingData  // Datas send back to remote control
{
  float angle;
  float omega;
  int speed;
  uint16_t P;
  uint16_t I;
  uint16_t D;
  uint16_t null_1;
  uint16_t null_2;
};
OutgoingData outgoing;



//***************************************************************************
//  setup - Set up the code
//***************************************************************************
void setup()
{
  Serial.begin(115200);      // Set up serial port for debug
  
  Wire.begin();

  // Initialize the timers
  
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
  
  pinMode(ENA, OUTPUT);
  
  pinMode(ENB, OUTPUT);
  pinMode(18, INPUT);
  pinMode(2, INPUT);

  //  Set up the interrupt handlers for the Left and Right Shaft Encoders
  //  Right encoder uses D19 (ISR4) for counts and D18 for direction
  //  Left encoder uses D3 (ISR1) for counts and D2 for direction
  
  attachInterrupt(4, RightEncoderISR, FALLING);
  attachInterrupt(1, LeftEncoderISR, FALLING);
  
  //---------------------------------------------
  // 24L01 initialization
  
  Mirf.cePin = 53;
  Mirf.csnPin = 48;
  Mirf.spi = &MirfHardwareSpi;  
  Mirf.init();
  Mirf.setRADDR((byte *)"serv1");
  Mirf.payload = 16;                    // Size of data packet
  Mirf.config();
}

void loop()
{
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
//******************************************************************
//  Receive 
//     - get data from the controller
//     - parse and scale the joystick values
//     - set up and send data back to the controller


void Receive()
{
  if (!Mirf.isSending() && Mirf.dataReady())
  {
    // Read datas from the romote controller
    Mirf.getData((byte *) &incoming);
    /*
    Serial.print("axis_1=");
    Serial.print(incoming.axis_1);
    Serial.print("  axis_2=");
    Serial.print(incoming.axis_2);
    Serial.print("  axis_3=");
    Serial.print(incoming.axis_3);
    Serial.print("  axis_4=");
    Serial.print(incoming.axis_4);
    Serial.print("  axis_5=");
    Serial.print(incoming.axis_5);
    Serial.print("  axis_6=");
    Serial.print(incoming.axis_6);
    Serial.print("  axis_7=");
    Serial.print(incoming.axis_7);
    Serial.print("  axis_8=");
    Serial.println(incoming.axis_8);
    */
    Mirf.setTADDR((byte *)"clie1");
    Mirf.send((byte *) &outgoing);  // Send data back to the controller
    
    //_______________________________________________________
    //  Axis 1 is right joystick Y axis (Forward/Back)
    //
    //  If abs(value) < 20 set to 0
    //  Else map input to -100 to +100
    
    if (incoming.axis_1 >= 520) // Y axis datas from joystick_1
    {
      Turn_Speed = map(incoming.axis_1, 520, 1023, 0, 120);
    }
    else if (incoming.axis_1 <= 480)
    {
      Turn_Speed = map(incoming.axis_1, 480 , 0, 0, -120);
    }
    else
    {
      Turn_Speed = 0;
    }

    //_______________________________________________________
    //  Axis 2 is right joystick X axis (Left/Right)
    //
    //  If abs(value) < 20 set to 0
    //  Else map input to -100 to +100

    if (incoming.axis_2 >= 520) // X axis datas from joystick_1
    {
      Run_Speed = map(incoming.axis_2, 520, 1023, 0, 100);
    }
    else if (incoming.axis_2 <= 480)
    {
      Run_Speed = map(incoming.axis_2, 480, 0, 0, -100);
    }
    else
    {
      Run_Speed = 0;
    }

  }
  else
  {
    incoming.axis_1 = incoming.axis_2 = 500;
  }
  
  // Put the structure values into the structure (data)
  //   for transmission back to the controller
  
  outgoing.omega = omega;
  outgoing.angle = Angle_Filtered;
  outgoing.speed = Sum_Right;
  outgoing.P = analogRead(A0);
  outgoing.I = analogRead(A1);
  outgoing.D = analogRead(A2);
}


//******************************************************************
//  Filter 
//     - get data from the accelerometer/gyro
//     - Calculate the angle and angular speed

void Filter()
{
  // Read raw data from the accelerometer/gyro
  
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Calculate the angle (0 = upright) in degrees
  
  Angle_Raw = (atan2(ay, az) * 180 / pi + Angle_offset);
  
  // Calculate the angular speed (gyro input) 
  
  omega = gx / Gyr_Gain + Gry_offset;
  
  // Calculate the elapsed time since the last measurment
  
  unsigned long now = micros();
  timeChange = now - preTime;
  preTime = now;
  dt = timeChange * 0.000001;     // dt is the elapsed time in seconds
  
  // Change in raw angle
  
  Angle_Delta = (Angle_Raw - Angle_Filtered) * 0.64; 

     
  Angle_Recursive = Angle_Delta * dt + Angle_Recursive; 
  
  
  Angle_Confidence = Angle_Recursive + (Angle_Raw - Angle_Filtered) * 1.6 + omega;
  
  
  Angle_Filtered = Angle_Confidence * dt + Angle_Filtered;
}

void myPID()
{
  kp = 22.000; 
  ki = 0;
  kd = 1.60;
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


//-------------------------------------------------------------
//  PWM Control  -  Set up the motor commands
//
//  LOutput is the desired speed of the left motor (-255 to +255)
//  ROutput is the desired speed of the right motor (-255 to +255)
//-------------------------------------------------------------

void PWMControl()

//-------------------------------------------------------------
//  Set the direction for the H-Bridge based upon the sign of
//    the desired speed
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
  
  //----------------------------------------------------------------
  // Duty cycle for the left motor is (4 * LOutput) / 1024 
  //    1023 is the full on value
  //
  // Timer/Counter3 is a general purpose 16-bit Timer/Counter module
  // 
  OCR3A = min(1023, (abs(LOutput * 4) + LMotor_offset * 4));
  
  //----------------------------------------------------------------
  // Duty cycle for the right motor is (ROutput) / 256 
  //    255 is the full on value
  //
  // Timer/Counter0 is a general purpose 8-bit Timer/Counter module
  //
  OCR0B = min(255, (abs(ROutput) + RMotor_offset)); 
}

//-------------------------------------------------------------
//  RightEncoderISR  -  ISR for right shaft encoder
//
//     Digital Input 18 is the direction input from the encoder
//-------------------------------------------------------------

void RightEncoderISR()
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

//-------------------------------------------------------------
// LeftEncoderISR  -  ISR for left shaft encoder
//
//     Digital Input 2 is the direction input from the encoder
//-------------------------------------------------------------

void LeftEncoderISR()
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

