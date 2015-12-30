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

// Define the points where each axis of the remote is centered

#define Axis1_Center  522
#define Axis2_Center  493
#define Axis3_Center  518
#define Axis4_Center  520

// Deadband is the area around center of joystick which is treated as 0

#define Deadband  3

// Thresholds at which the wheels start to move

#define RightThreshold  35
#define LeftThreshold   50

float Angle_Delta, Angle_Recursive, Angle_Confidence;

float kp, ki, kd;     // PID constants
float Angle_Raw, Angle_Filtered;
float Gyro_Raw, Gyro_Filtered;
float omega, dt;
float Turn_Speed = 0;    // Joystick X value mapped to -100 to +100 with deadband
float Run_Speed = 0;     // Joystick Y value mapped to -120 to +120 with deadband
int   LOutput, ROutput;
float Input, Output;
float Output_Filtered;
float AngleVertical = 3.0; // Angle at which robot is stationary upright
float AngleTarget = 0.0;   // Desired angle in degrees.
int   outputindex = 0;


unsigned long preTime, lastTime;
float errSum, dErr, error, lastErr;
int timeChange;

//int filtercount = 0;


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
  uint16_t spare1;
  uint16_t spare2;
  uint16_t spare3;
  uint16_t spare4;
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



//***************************************************************************
//  setup - Set up the code
//***************************************************************************
void setup()
{
  Serial.begin(115200);      // Set up serial port for debug
  
  Wire.begin();


  /* If the robot was turned on with the angle over 45(-45) degrees,the wheels
   will not spin until the robot is in right position. */
  accelgyro.initialize();
  for (int i = 0; i < 200; i++) // Looping 200 times to get the real gesture when starting
  {
    GetValues();
    Filter();
  }
  if (abs(Angle_Filtered) < 45)  // Start to work after cleaning data
  {
    omega = Angle_Raw = Angle_Filtered = 0;
    Output = error = errSum = dErr = 0;
    Filter();
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
  
  //attachInterrupt(4, RightEncoderISR, FALLING);
  //attachInterrupt(1, LeftEncoderISR, FALLING);
  
  //---------------------------------------------
  // 24L01 initialization
  
  Mirf.cePin = 53;
  Mirf.csnPin = 48;
  Mirf.spi = &MirfHardwareSpi;  
  Mirf.init();
  Mirf.setRADDR((byte *)"serv1");
  Mirf.payload = 24;                    // Size of received data packet
  Mirf.config();
  Serial.println("Initialized");
  
}

void loop()
{
  while (1)
  {
    Receive();
    
    // Only do this stuff if at least 10 milliseconds has passed since last time
    
    if ((millis() - lastTime) > 10)
    {
      GetValues();
      Filter();
      
      // If angle > 45 or < -45 then stop the robot
      if (abs(Angle_Filtered) < 45)
      {
        DonsPID();
        if ((outputindex % 1) == 0)
        {
          
          PWMControl();
        }
      }
      else
      {
          Serial.println("Tilt");
          LOutput = 0;
          ROutput = 0;
          PWMControl();
 //       digitalWrite(TN1, HIGH);
 //       digitalWrite(TN2, HIGH);
 //       digitalWrite(TN3, HIGH);
 //       digitalWrite(TN4, HIGH);
      }
      lastTime = millis();
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
  int temp; 
  
  
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
    //  Axis 2 is right joystick X axis (Left/Right)
    //
    //  If abs(value) < 20 set to 0
    //  Else map input to -100 to +100
    
    temp = incoming.axis_2;
    temp = temp - Axis2_Center;
    if (abs(temp) < Deadband)
    {
      temp = 0;
    }
    
    Turn_Speed = (float)temp;
    

    //_______________________________________________________
    //  Axis 1 is right joystick Y axis (Forward/Back)
    //
    //  If abs(value) < 20 set to 0
    //  Else map input to -100 to +100

    temp = incoming.axis_1;
    temp = temp - Axis1_Center;
    if (abs(temp) < Deadband)
    {
      temp = 0;
    }
    
    Run_Speed = (float)temp;
  }

//      Run_Speed = map(incoming.axis_1, 520, 1023, 0, 100);
  
  // Read the analog potentiometers and convert the values to kp, ki, kd
  
  kp = float(analogRead(A0))/10.0;
  ki = float(analogRead(A1))/100.0;
  kd = float(analogRead(A2))/100.0;
  
  // Put the structure values into the structure (data)
  //   for transmission back to the controller
  
  outgoing.omega = omega;
  outgoing.angle = Angle_Filtered;
  outgoing.speed = (int)ROutput;
  outgoing.P = kp; 
  outgoing.I = ki;
  outgoing.D = kd;
  
  //Serial.print("kp = ");
  //Serial.print(kp);
  //Serial.print(" kd = ");
  //Serial.println(kd);
}

//******************************************************************
//  GetValues 
//     - get data from the accelerometer/gyro
//     - Calculate the angle and angular speed

void GetValues()
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
}


//******************************************************************
//  Filter 
//     - get data from the accelerometer/gyro
//     - Calculate the angle and angular speed

#define FILTERSIZE  5
int   index = 0;
float angleFilter[FILTERSIZE];
float gyroFilter[FILTERSIZE];

void Filter()
{ 
  float total = 0.0;
  
  angleFilter[index % FILTERSIZE] = Angle_Raw;
  for (int i = 0; i < FILTERSIZE; i++)
  {
    total += angleFilter[i];
  }
  Angle_Filtered = total / (float)FILTERSIZE;
  
  total = 0.0;
  gyroFilter[index % FILTERSIZE] = Gyro_Raw;
  for (int i = 0; i < FILTERSIZE; i++)
  {
    total += gyroFilter[i];
  }
  Gyro_Filtered = total / (float)FILTERSIZE;\
  
  index++;
}



//******************************************************************
//  Don's PID 
//     - Use the desired angle (AngleTarget) and the current angle (Angle_Raw)
//        to feed the P part of PID
//     - Use the gyro value (omega) to feed the D part of PID
//     - With P and D determine wheel speed (common to both wheels)
//        to balance the robot at AngleTarget
//     - Adjust right and left wheel speed based upon
//        1. The X axis of the joystick (desired turn)
//        2. Accumulated error of left/right encoders
//            to keep the robot pointed straight
//     - Feed these values to the motors (LOutput and ROutput)


void DonsPID(){
  
  int turnbias;
  
  // First calculate the desired target angle based upon the position of the joystick
  
  AngleTarget = AngleVertical + Run_Speed * 0.1;
  
  // Next calculate the raw value needed to keep the robot balanced
  //   at the desired target angle 
  
  Output = kp * (Angle_Raw - AngleTarget) + kd * (omega);
  
  // Next, calculate the bias based upon the right/left bias from the joystick

  turnbias = (int) (Turn_Speed * .1);
  
  OutputFilter();
  //Output = Run_Speed;
  
  noInterrupts();
  ROutput = (int) (Output_Filtered * .55);
  LOutput = (int) (Output_Filtered);
  
  //ROutput = (int)Output_Filtered + turnbias;
  //LOutput = (int)Output_Filtered - turnbias;
  interrupts();
  
  
  
}
//******************************************************************
//  Output Filter 
//     - Filter and average the last N output values

#define OUTPUTFILTERSIZE  5
float outputFilter[OUTPUTFILTERSIZE];

void OutputFilter()
{ 
  float total = 0.0;
  
  outputFilter[outputindex % OUTPUTFILTERSIZE] = Output;
  for (int i = 0; i < OUTPUTFILTERSIZE; i++)
  {
    total += outputFilter[i];
  }
  Output_Filtered = total / (float)OUTPUTFILTERSIZE;
  
  outputindex++;
}





//-------------------------------------------------------------
//  PWM Control  -  Set up the motor commands
//
//  LOutput is the desired speed of the left motor (-255 to +255)
//  ROutput is the desired speed of the right motor (-255 to +255)
//-------------------------------------------------------------

void PWMControl()
{
  
  int actualPWM;
  
  //-------------------------------------------------------------
  //  Set the direction for the H-Bridge based upon the sign of
  //    the desired speed; 
  //  Cap the desired output at +/- 255

  if (LOutput > 0)
  {
    digitalWrite(TN1, HIGH);
    digitalWrite(TN2, LOW);
    LOutput = min (255, LOutput);
  }
  else if (LOutput < 0)
  {
    digitalWrite(TN1, LOW);
    digitalWrite(TN2, HIGH);
    LOutput = max (-255, LOutput);

  }
 
 if (ROutput > 0)
  {
    digitalWrite(TN3, HIGH);
    digitalWrite(TN4, LOW);
    ROutput = min (255, ROutput);
  }
  else if (ROutput < 0)
  {
    digitalWrite(TN3, LOW);
    digitalWrite(TN4, HIGH);
    ROutput = max (-255, ROutput);
  }
  
  // At this point, ROutput and LOutput are in the range -255 to +255
  // Now we will scale these values to the range which actually moves
  //   the wheels. 
  
  actualPWM = map(abs(ROutput), 0, 255, RightThreshold, 255);  
  analogWrite(ENA, actualPWM);
  
  
  actualPWM = map(abs(LOutput), 0, 255, LeftThreshold, 255);  
  analogWrite(ENB, actualPWM);
  
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

