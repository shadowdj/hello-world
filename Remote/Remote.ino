// SainSmart Instabots Upright Rover rev. 2.0
// http://www.sainsmart.com

#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3F, 16, 2);

unsigned int Display_Counter, Button_Delay = 0;

int Center_axis_1;
int Center_axis_2;
int Center_axis_3;
int Center_axis_4;

struct Axis {
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
Axis axis_x;

struct Gesture {
  float angle;
  float omega;
  int speed;
  float P;
  float I;
  float D;
  uint16_t null_1;
  uint16_t null_2;
};
Gesture data;

void setup() {
  pinMode(2, INPUT);
  lcd.init();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("SainSmartProduct");
  lcd.setCursor(0, 1);
  lcd.print("Remote V 2.0");

  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  Mirf.setRADDR((byte *)"clie1");
  //Mirf.payload = sizeof(struct Gesture);
  Mirf.payload = 24;
  Mirf.config();
  
  Center_axis_1 = analogRead(A0);
  Center_axis_2 = analogRead(A1);
  Center_axis_3 = analogRead(A2);
  Center_axis_4 = analogRead(A3);


  delay(4000);
}

void loop() {
  unsigned long time = millis();
  /**********************************************************************************************************/
  axis_x.axis_1 = analogRead(A0) - Center_axis_1;
  axis_x.axis_2 = analogRead(A1) - Center_axis_2;
  axis_x.axis_3 = analogRead(A2) - Center_axis_3;
  axis_x.axis_4 = analogRead(A3) - Center_axis_4;
  
  if (abs(axis_x.axis_1) < 3)
  {
    axis_x.axis_1 = 0;
  }
  
  if (abs(axis_x.axis_2) < 3)
  {
    axis_x.axis_2 = 0;
  }
  
  if (abs(axis_x.axis_3) < 3)
  {
    axis_x.axis_3 = 0;
  }
  
  if (abs(axis_x.axis_3) < 3)
  {
    axis_x.axis_4 = 0;
  }
  
  Mirf.setTADDR((byte *)"serv1");
  Mirf.send((byte *)&axis_x);
  while (Mirf.isSending()) {
  }
  /**********************************************************************************************************/
  while (!Mirf.dataReady()) {
    if ( ( millis() - time ) > 2000) {
      lcd.setCursor(0,0);
      lcd.print("   Waiting...   ");
      lcd.setCursor(0,1);
      lcd.print("                ");
      return;
      }
    }
    Mirf.getData((byte *) &data);
    /**********************************************************************************************************/
    digitalWrite(2, HIGH);
    while (digitalRead(2) == LOW) {
      Button_Delay++;
      delay(1);
    }

    if (Button_Delay > 10) {
      lcd.clear();
      Display_Counter++;
    }
    Button_Delay = 0;

    while (digitalRead(2) == LOW) {
      delay(1);
    }

    if (Display_Counter & 1) {
      PID_Display();
    }
    else {
      Gesture_Display();
    }
    /**********************************************************************************************************/
    /*Serial.print("Ping:");
    Serial.println((millis() - time));*/
  }
  /**********************************************************************************************************/
  void PID_Display()
  {
    lcd.setCursor(0, 0);
    lcd.print("Parameter:P=");
    //if (data.P < 1000) {
    // lcd.print('0');
    //  if (data.P < 100) {
    //    lcd.print('0');
    //    if (data.P < 10) {
    //      lcd.print('0');
    //    }
    //  }
    //}
    lcd.print(data.P);
    /**********************************************************************************************************/
    lcd.setCursor(0, 1);
    lcd.print(" I=");
    //if (data.I < 1000) {
    //  lcd.print('0');
    //  if (data.I < 100) {
     //   lcd.print('0');
     //   if (data.I < 10) {
    //      lcd.print('0');
    //    }
    //  }
    //}
    lcd.print(data.I);
    /**********************************************************************************************************/
    lcd.print("  D=");
    //if (data.D < 1000) {
    //  lcd.print('0');
    // if (data.D < 100) {
    //    lcd.print('0');
    //    if (data.D < 10) {
    //      lcd.print('0');
    //    }
    //  }
    //}
    lcd.print(data.D);
  }
  /**********************************************************************************************************/
  void Gesture_Display()
  {
    lcd.setCursor(0, 0);
    lcd.print("Gesture:A=");
    if (10 <= data.angle) {
      lcd.print("+");
      lcd.print(data.angle);
    }
    if (0 < data.angle && data.angle < 10) {
      lcd.print("+0");
      lcd.print(data.angle);
    }
    if (-10 < data.angle && data.angle < 0) {
      data.angle = -data.angle;
      lcd.print("-0");
      lcd.print(data.angle);
    }
    if (data.angle <= -10) {
      lcd.print(data.angle);
    }
    /**********************************************************************************************************/
    lcd.setCursor(0, 1);
    lcd.print("O=");
    if (100 <= data.omega) {
      lcd.print("+");
      lcd.print(data.omega);
    }
    if (10 <= data.omega && data.omega < 100) {
      lcd.print("+0");
      lcd.print(data.omega);
    }
    if (0 < data.omega && data.omega < 10) {
      lcd.print("+00");
      lcd.print(data.omega);
    }
    if (-10 < data.omega && data.omega <= 0) {
      data.omega = -data.omega;
      lcd.print("-00");
      lcd.print(data.omega);
    }
    if (-100 < data.omega && data.omega <= -10) {
      data.omega = -data.omega;
      lcd.print("-0");
      lcd.print(data.omega);
    }
    if (data.omega <= -100) {
      lcd.print(data.omega);
    }
    /**********************************************************************************************************/
    lcd.setCursor(9, 1);
    lcd.print(" S=");
    if (data.speed <= -100) {
      lcd.print(data.speed);
    }
    if (-100 < data.speed && data.speed <= -10) {
      data.speed = -data.speed;
      lcd.print("-0");
      lcd.print(data.speed);
    }
    if (-10 < data.speed && data.speed <= 0) {
      data.speed = -data.speed;
      lcd.print("-00");
      lcd.print(data.speed);
    }
    if (0 < data.speed && data.speed < 10) {
      lcd.print("+00");
      lcd.print(data.speed);
    }
    if (10 <= data.speed && data.speed < 100) {
      lcd.print("+0");
      lcd.print(data.speed);
    }
    if (100 <= data.speed) {
      lcd.print("+");
      lcd.print(data.speed);
    }
  }

