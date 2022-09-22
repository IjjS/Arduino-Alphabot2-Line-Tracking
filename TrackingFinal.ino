#include "TRSensors.h"
#include <Wire.h>

#define PWMA   6            //Left motor speed control with PWM signal.
#define AIN2   A0           //Left motor forward.
#define AIN1   A1           //Left motor backward.
#define PWMB   5            //Right motor speed control with PWM signal.
#define BIN1   A2           //Right motor forward, the same direction as the left motor backward.
#define BIN2   A3           //Right motor backward, the same direction as the left motor forward.
#define PIN 7
#define NUM_SENSORS 5       //Five IR sensors under the Base of the Alphabot-2.
#define Addr 0x20           //Communication address for the joystick.


//Global variables to be stored regardless of the loop function
TRSensors trs =   TRSensors();        
unsigned int sensorValues[NUM_SENSORS];        
unsigned int error_prior = 0;                           
unsigned int position;                            
long integral = 0;                              
uint16_t i, j, k, m;
byte value;
bool FOUND = true;                              //Boolean data type variable to ensure the robot is on the line.

void PCF8574Write(byte data);                   //Communication for the joystick.
byte PCF8574Read();

void setup() {
  Serial.begin(115200);                         //Serial communication begins with 115200 bauds.
  Wire.begin();                                 //Communication with the joystick.

  while (value != 0xEF) {                       //If the joystick button is not pressed,
    PCF8574Write(0x1F | PCF8574Read());         //Waits until the press is detected.
    value = PCF8574Read() | 0xE0;               //0xEF = 0b11101111, the button pressed.
  }

  pinMode(PWMA, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN2, HIGH);
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, 80);
  analogWrite(PWMB, 80);

  for (int i = 0; i < 100; i++) {               //Make the calibration take about 10 seconds
    if (i < 25 || i >= 75)
    {
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
    }
    else {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
    }
    trs.calibrate();                            
  }                                             
  analogWrite(PWMA, 0);                         //After the calibration is done, stop and wait.
  analogWrite(PWMB, 0);
  digitalWrite(AIN2, LOW);
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);

  value = 0;
  
  //Wait until the joystick button is pressed.
  while (value != 0xEF) {                   
    PCF8574Write(0x1F | PCF8574Read());
    value = PCF8574Read() | 0xE0;
    position = trs.readLine(sensorValues) / 200;    
    delay(100);                                     
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }
}

void loop() {
  trs.readLine(sensorValues);                 //Read the sensor values for every loop.

  //All the sensors read bright
  if (sensorValues[0] < 20 && sensorValues[1] < 20 && sensorValues[2] < 20 && sensorValues[3] < 20 && sensorValues[4] < 20) {
    FOUND = false;
    analogWrite(PWMA, 0);                     
    analogWrite(PWMB, 0);                     
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMA, 60);                    //Move backward slightly to check the last position on the line.
    analogWrite(PWMB, 60);
    delay(100);
    position = trs.readLine(sensorValues);    //Read the last position.
  }

  //If the last position was the same as the desired position, move forward with searching every 1 millisecond for the next line.
  if (FOUND == false && position == 2000) {       
    digitalWrite(AIN1, LOW);                      
    digitalWrite(AIN2, HIGH);                     
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMA, 180);
    analogWrite(PWMA, 180);
    //delay(50);

    //Path searching for 100 ms.
    for (int i=0; i < 100; i++) {
      delay(1);
      trs.readLine(sensorValues);
      if (sensorValues[0] > 50 || sensorValues[1] > 50 || sensorValues[2] > 50 || sensorValues[3] > 50 || sensorValues[4] > 50) {
        FOUND = true;
        error_prior = 0;                        //If any of five sensors reads a color, get ready to restart the PID control by initializing the integral and prior error value.
        integral = 0;                           
        break;
      }
    }
  }

  //If the robot was supposed to go to the right before derailing, turn right slightly and move forward to search.
  if (FOUND == false && position > 2000) {        
    digitalWrite(AIN1, LOW);            
    digitalWrite(AIN2, HIGH);                  

    analogWrite(PWMA, 90);
    analogWrite(PWMB, 90);
    delay(100);

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);

    analogWrite(PWMA, 100);
    analogWrite(PWMB, 100);
    //delay(50);

    //Searching right
    for (int j = 0; j < 40; j++) {
      delay(1);
      trs.readLine(sensorValues);
      if (sensorValues[0] > 50 || sensorValues[1] > 50 || sensorValues[2] > 50 || sensorValues[3] > 50 || sensorValues[4] > 50) {
        FOUND = true;
        error_prior = 0;                          //Initialize the PID function. Otherwise, it will rotate at
        integral = 0;                             //one spot as integral is a hugely large number.
        Break;                                   
      }
    }

    if (FOUND == false) {                         //If the robot cannot find the line by forward searching,
      analogWrite(PWMA, 10);                      //begin to search by move circularly because it can find the line as the rail was broken
      analogWrite(PWMB, 130);                     //somewhere at the right of the last position.
      for (int k = 0; k < 300; k++) {
        delay(1);
        trs.readLine(sensorValues);
        if (sensorValues[0] > 50 || sensorValues[1] > 50 || sensorValues[2] > 50 || sensorValues[3] > 50 || sensorValues[4] > 50) {
          analogWrite(PWMA, 0);
          analogWrite(PWMB, 0);
          FOUND = true;
          error_prior = 0;
          integral = 0;
          break;
        }
      }
    }
  }

  //If the robot was supposed to go to the left, turn left and do the forward searching for a moment.
  if (FOUND == false && position < 2000) {        
    digitalWrite(BIN1, LOW);                     
    digitalWrite(BIN2, HIGH);                     

    analogWrite(PWMA, 90);
    analogWrite(PWMB, 90);
    delay(100);

    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);

    analogWrite(PWMA, 100);
    analogWrite(PWMB, 100);
    //delay(50);

    //Searching left
    for (int m = 0; m < 40; m++) {
      delay(1);
      trs.readLine(sensorValues);
      if (sensorValues[0] > 50 || sensorValues[1] > 50 || sensorValues[2] > 50 || sensorValues[3] > 50 || sensorValues[4] > 50) {
        FOUND = true;
        error_prior = 0;
        integral = 0;
        break;
      }
    }

    //if the path not found, circular search
    if (FOUND == false) {                         
      analogWrite(PWMA, 130);                     
      analogWrite(PWMB, 10);
      for (int n = 0; n < 300; n++) {
        delay(1);
        trs.readLine(sensorValues);
        if (sensorValues[0] > 50 || sensorValues[1] > 50 || sensorValues[2] > 50 || sensorValues[3] > 50 || sensorValues[4] > 50) {
          analogWrite(PWMA, 0);             //Initialize the PID function.
          analogWrite(PWMB, 0);
          FOUND = true;
          error_prior = 0;
          integral = 0;
          break;
        }
      }
    }
  }

  position = trs.readLine(sensorValues);            //Decide position to run when the robot finds the line.
  
  if (FOUND == true) {                              
    int error, derivative, m_control;               
 
    error = (int)position - 2000;                   //Error feedback.
    integral += error;                              
    derivative = error - error_prior;               
    error_prior = error;

    //PID: Kp, Ki, and Kd values obtained by some trials of tuning.
    m_control = 0.075 * error + 0.0001 * integral + 20 * derivative; 
    const int maxspeed = 180;                                        

    if (m_control > maxspeed)                                     //PID control function may result in hugely high or low value, so we have to convert it to the maximum speed value.
      m_control = maxspeed;                                      
    if (m_control < - maxspeed)
      m_control = - maxspeed;

    /*
     * While following the path, there may be some distracts, such as a horizontal path across the original path.
     * The original line is 2cm wide, so only the one sensor at the center fully reads the high value.
     */
    if (sensorValues[1] > 900 && sensorValues[2] > 900 && sensorValues[3] > 900) {   
      analogWrite(PWMA, maxspeed);                                  
      analogWrite(PWMB, maxspeed);                                   
    }                                                                
    if (m_control > 0) {
      analogWrite(PWMA, maxspeed);                                  
      analogWrite(PWMB, maxspeed - m_control);                       
    }
    else {
      analogWrite(PWMA, maxspeed + m_control);
      analogWrite(PWMB, maxspeed);
    }
  }
}

//PCF8574 is a chip to transmit and convert the input value of the joystick to the digital value.
void PCF8574Write(byte data) {                                        
  Wire.beginTransmission(Addr);                                     
  Wire.write(data);
  Wire.endTransmission();
}


byte PCF8574Read() {
  int data = -1;
  Wire.requestFrom(Addr, 1);
  if (Wire.available()) {
    data = Wire.read();
  }
  return data;
}
