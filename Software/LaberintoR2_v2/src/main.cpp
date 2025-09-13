#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <BluetoothSerial.h>
#include <motors_TB6612fng.h>
#include "I2Cdev.h"
#include "MPU6050.h"

//Distance Sensors
#define PIN_ECHO_R        39 
#define PIN_ECHO_L        35 
#define PIN_ECHO_FRONT    34 
#define PIN_TRIGGER_R     32 
#define PIN_TRIGGER_L     33 
#define PIN_TRIGGER_FRONT 25 
//Switches
#define PIN_START 17
#define PIN_RIGHT 18
#define PIN_LEFT  19
//Motor Drivers
#define PIN_B1    27
#define PIN_B2    26
#define PIN_A1     2
#define PIN_A2    13
#define PIN_PWM_B  4
#define PIN_PWM_A 16

#define CHANNEL_MR 0
#define CHANNEL_ML 1 
#define PWM_FREQUENCY 1000 
#define PWM_RESOLUTION 8

MotorPair motor (PIN_A1 , PIN_A2 , PIN_PWM_A , CHANNEL_MR ,
                 PIN_B1 , PIN_B2 , PIN_PWM_B , CHANNEL_ML ,
                 PWM_FREQUENCY , PWM_RESOLUTION);


//Bluetooth
const char *pin = "1234";
String device_name = "Romano v2";
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif
BluetoothSerial SerialBT;
int dato_BT;


// La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo 
// del estado de AD0. Si no se especifica, 0x68 estará implicito
MPU6050 sensor;

// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
int16_t gx, gy, gz;

long tiempo_prev, dt;
float girosc_ang_x, girosc_ang_y;
float girosc_ang_x_prev, girosc_ang_y_prev;



void setup() {
  Serial.begin (115200);

  //Switches
  pinMode (PIN_START , INPUT_PULLDOWN);
  pinMode (PIN_LEFT  , INPUT_PULLDOWN);
  pinMode (PIN_RIGHT , INPUT_PULLDOWN);

  //Distance Sensors
  pinMode (PIN_ECHO_R , INPUT);
  pinMode (PIN_ECHO_L , INPUT);
  pinMode (PIN_ECHO_FRONT , INPUT);
  pinMode (PIN_TRIGGER_R  , OUTPUT);
  pinMode (PIN_TRIGGER_L  , OUTPUT);
  pinMode (PIN_TRIGGER_FRONT , OUTPUT);

  digitalWrite (PIN_TRIGGER_R , LOW);
  digitalWrite (PIN_TRIGGER_L , LOW);
  digitalWrite (PIN_TRIGGER_FRONT , LOW);


  //Bluetooth
  SerialBT.begin(device_name);
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());

  #ifdef USE_PIN
    SerialBT.setPin(pin);
    Serial.println("Using PIN");
  #endif

  //MPU6050
  Wire.begin();       
  sensor.initialize(); 
  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");
  tiempo_prev=millis();
}


void Bluetooth() {
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }
  delay(20);

  dato_BT = SerialBT.read();
}



int Read_MPU6050() {
  // Leer las velocidades angulares 
  sensor.getRotation(&gx, &gy, &gz);
  
  //Calcular los angulos rotacion:
  
  dt = millis()-tiempo_prev;
  tiempo_prev=millis();
  
  girosc_ang_x = (gx/131)*dt/1000.0 + girosc_ang_x_prev;
  girosc_ang_y = (gy/131)*dt/1000.0 + girosc_ang_y_prev;

  girosc_ang_x_prev=girosc_ang_x;
  girosc_ang_y_prev=girosc_ang_y;

  //Mostrar los angulos separadas por un [tab]
  Serial.print("Rotacion en X:  ");
  Serial.print(girosc_ang_x); 
  Serial.print("tRotacion en Y: ");
  Serial.println(girosc_ang_y);
  delay(10);

  return girosc_ang_x;
}


#define MAX_LEFT       255
#define MAX_RIGHT      190
#define MID_SPEED      140
#define LOW_SPEED       70
#define BREAK            0
#define WALL_DETECTED_FRONT  500
#define WALL_DETECTED_SIDES 1000
#define SIDE_SETPOINT 500
#define SETPOINT 25

int Read_Front_Sensor () {
  digitalWrite (PIN_TRIGGER_FRONT , HIGH);
  delayMicroseconds (10);
  digitalWrite (PIN_TRIGGER_FRONT , LOW);
  int time = pulseIn (PIN_ECHO_FRONT , HIGH);
  
  return time;
}

int Read_Left_Sensor () {
  digitalWrite (PIN_TRIGGER_L , HIGH);
  delayMicroseconds (10);
  digitalWrite (PIN_TRIGGER_L , LOW);
  int time = pulseIn (PIN_ECHO_L , HIGH);

  return time;
}

int Read_Right_Sensor () {
  digitalWrite (PIN_TRIGGER_R , HIGH);
  delayMicroseconds (10);
  digitalWrite (PIN_TRIGGER_R , LOW);
  int time = pulseIn (PIN_ECHO_R , HIGH);

  return time;
}




bool front_wall_detected (int sensor) {
  if (sensor < WALL_DETECTED_FRONT) {
    return true;
  } else {
    return false;
  }
}

bool side_wall_detected (int sensor) {
  if (sensor < WALL_DETECTED_SIDES) {
    return true;
  } else {
    return false;
  }
}

bool side_setpoint_reached (int sensor) {
  if (sensor <= SIDE_SETPOINT) {
    return true;
  } else {
    return false;
  }
}


bool button_order (int button) {
  int button_read = digitalRead (button);

  if (button_read == LOW) {
    return true;
  } else {
    return false;
  }
}




int setPoint = SETPOINT;
int position;

int proportional;
int derivative;
int integral;
int lastError;

int maxSpeed = 255;
int minSpeed = 205;
int speed    = 235;

float kp = 0.70;
float ki =    0;
float kd = 0.18;
float pid;
float pidRight;
float pidLeft;

#define RIGHT_MOTOR_PWM_ERROR 65

void PID (int wall_sensor) {

  position = wall_sensor;

  proportional = position - setPoint;
  derivative = proportional - lastError;

  pid = (proportional * kp) + (derivative * kd);

  lastError = proportional;

  pidRight = speed + pid;
  pidLeft = speed - pid;

  if (pidRight > maxSpeed) {
    pidRight = maxSpeed;
  }
  if (pidLeft > maxSpeed) {
    pidLeft = maxSpeed;
  }

  if (pidRight <= minSpeed && pidLeft > minSpeed){
    motor.TurnRight(minSpeed + (minSpeed - pidRight), pidLeft);
  }
  else if (pidLeft <= minSpeed && pidRight > minSpeed) {
    motor.TurnLeft(pidRight, minSpeed + (minSpeed - pidLeft));
  }
  else {
    motor.MoveForwards(pidRight - RIGHT_MOTOR_PWM_ERROR , pidLeft);
  }
}

bool robot_is_turning = false;
bool turning_left  = false;
bool turning_right = false;

void TurnLeftPerLeft (int left_sensor) {
  motor.TurnLeft (LOW_SPEED , MAX_RIGHT);

  if (side_setpoint_reached (left_sensor)) {
    turning_left = false;
  }
}

void TurnRightPerLeft (int left_sensor) {
  motor.TurnRight (MAX_LEFT , BREAK);

  if (side_setpoint_reached (left_sensor)) {
    turning_right = false;
  }
}

void TurnRightPerRight (int right_sensor) {
  motor.TurnRight (MAX_LEFT , LOW_SPEED);

  if (side_setpoint_reached (right_sensor)) {
    turning_right = false;
  }
}

void TurnLeftPerRight (int right_sensor) {
  motor.TurnLeft (BREAK , MAX_RIGHT);

  if (side_setpoint_reached (right_sensor)) {
    turning_right = false;
  }
}






void FollowLeftWall () {
  int front = Read_Front_Sensor();
  int left  =  Read_Left_Sensor();

  if (!robot_is_turning) {
    if (side_wall_detected (left)) {
      if (front_wall_detected (front)) {
        robot_is_turning = true;
        turning_right = true;
      }
      else {
        PID (left);
      }
    }
    else {
      robot_is_turning = true;
      turning_left = true;
    }
  }

  else {

    if (turning_left) {
      TurnLeftPerLeft (left);
      if (!turning_left) {
        robot_is_turning = false;
      }
    }

    if (turning_right) {
      TurnRightPerLeft (left);
      if (!turning_right) {
        robot_is_turning = false;
      }
    }
  }


  if (front < 340) {
    motor.MoveBackwards (MAX_LEFT , MAX_RIGHT);
    delay (150);
      motor.TurnRight (MAX_LEFT , BREAK);
      delay (150);
  }
}


void FollowRightWall () {
  int front = Read_Front_Sensor();
  int right = Read_Right_Sensor();

  if (!robot_is_turning) {
    if (side_wall_detected (right)) {
      if (front_wall_detected (front)) {
        robot_is_turning = true;
        turning_right = true;
      }
      else {
        PID (right);
      }
    }
    else {
      robot_is_turning = true;
      turning_right = true;
    }
  }

  else {
    if (turning_left) {
      TurnLeftPerRight (right);
      if (!turning_left) {
        robot_is_turning = false;
      }
    }

    if (turning_right) {
      TurnRightPerRight (right);
      if (!turning_right) {
        robot_is_turning = false;
      }
    }
  }

  if (front < 340) {
    motor.MoveBackwards (MAX_LEFT , MAX_RIGHT);
    delay (150);
      motor.TurnLeft (BREAK , MAX_RIGHT);
      delay (125);
  }
}



bool waiting = false;
#define WAITING_TIME 2000

enum preparation_stage {
  PREPARATION,
  WAIT,
};
static preparation_stage turn_stage = PREPARATION;
static unsigned long action_start = 0;

void WaitTime () {

  unsigned long time = millis();

  if (turn_stage == PREPARATION) {
    Serial.println ("PREPARATION");
    action_start = time;
    turn_stage = WAIT;
  }

  if (turn_stage == WAIT) {
    Serial.println ("TURN U");
    motor.TurnAroundLeft (BREAK, BREAK);
    
    if (time - action_start >= WAITING_TIME) {
      turn_stage = PREPARATION;
      waiting = false;
      motor.MoveForwards (0 , 0);
      Serial.print ("WAITING");
    }
  }
}

bool mode_selected = false; 
bool go_per_left   = false;
bool go_per_right  = false;
bool ready         = false;





unsigned long waitStart = 0;
bool waitingStart = false;

void loop() {
  Bluetooth();

  if (button_order(PIN_LEFT)) {
    mode_selected = true;
    go_per_left   = true;
    go_per_right  = false;
  }
  else if (button_order(PIN_RIGHT)) {
    mode_selected = true;
    go_per_right  = true;
    go_per_left   = false;
  }
  else if (button_order(PIN_START)) {
    ready = true;
    waitingStart = true;         
    waitStart = millis();         
    motor.MoveForwards(0,0);
  }

  if (ready) {
    if (waitingStart) {
    
      if (millis() - waitStart >= 2000) {
        waitingStart = false;   
      } else {
        return; 
      }
    }

    
    if (mode_selected) {
      if (go_per_left) {
        FollowLeftWall();
      }
      else if (go_per_right) {
        FollowRightWall();
      }
    }
    else {
      motor.MoveForwards(MAX_LEFT , MAX_RIGHT);
    }
  }
}


















//XD