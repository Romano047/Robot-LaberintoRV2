#include <Arduino.h>
#include <Wire.h>
#include <BluetoothSerial.h>
#include <motors_TB6612fng.h>

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
#define PIN_B1    26
#define PIN_B2    27
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

/*
//MPU6050
//Direccion I2C de la IMU
#define MPU 0x68
 
//Ratios de conversion
#define A_R 16384.0 // 32768/2
#define G_R 131.0 // 32768/250
 
//Conversion de radianes a grados 180/PI
#define RAD_A_DEG 57.295779
 
//MPU-6050 da los valores en enteros de 16 bits
//Valores RAW
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
 
//Angulos
float Acc[2];
float Gy[3];
float Angle[3];

String valores;

long tiempo_prev;
float dt;
*/


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

  /*
  //MPU6050
  Wire.begin(21,22);
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  tiempo_prev = millis();
  */

  //Bluetooth
  SerialBT.begin(device_name);
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());

  #ifdef USE_PIN
    SerialBT.setPin(pin);
    Serial.println("Using PIN");
  #endif
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






/*void Read_MPU6050 () {

  unsigned long tiempo_actual = millis();
  dt = (tiempo_actual - tiempo_prev) / 1000.0; // dt en segundos
  tiempo_prev = tiempo_actual;

  //Leer los valores del Acelerometro de la IMU
   Wire.beginTransmission(MPU);
   Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,6,true);   //A partir del 0x3B, se piden 6 registros
   AcX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
   AcY=Wire.read()<<8|Wire.read();
   AcZ=Wire.read()<<8|Wire.read();
 
   //A partir de los valores del acelerometro, se calculan los angulos Y, X
   //respectivamente, con la formula de la tangente.
   Acc[1] = atan(-1*(AcX/A_R)/sqrt(pow((AcY/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
   Acc[0] = atan((AcY/A_R)/sqrt(pow((AcX/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
 
   //Leer los valores del Giroscopio
   Wire.beginTransmission(MPU);
   Wire.write(0x43);
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,6,true);   //A partir del 0x43, se piden 6 registros
   GyX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
   GyY=Wire.read()<<8|Wire.read();
   GyZ=Wire.read()<<8|Wire.read();
 
   //Calculo del angulo del Giroscopio
   Gy[0] = GyX/G_R;
   Gy[1] = GyY/G_R;
   Gy[2] = GyZ/G_R;
 
   //Aplicar el Filtro Complementario
   Angle[0] = 0.98 *(Angle[0]+Gy[0]*dt) + 0.02*Acc[0];
   Angle[1] = 0.98 *(Angle[1]+Gy[1]*dt) + 0.02*Acc[1];

   //Integración respecto del tiempo paras calcular el YAW
   Angle[2] = Angle[2]+Gy[2]*dt;
   int yaw;
   yaw = Angle[2];
 
   //Mostrar los valores por consola
   /*valores = "90, " +String(Angle[0]) + "," + String(Angle[1]) + "," + String(Angle[2]) + ", -90"; //Angle2 YAW IZQ -90 DER 90
   Serial.println(valores);
   delay(10);
}*/


#define MAX_SPEED 255
#define MID_SPEED 100
#define LOW_SPEED 50
#define BREAK 0
#define WALL_DETECTED 1200


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

bool wall_detected (int sensor) {
  if (sensor < WALL_DETECTED) {
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




int setPoint = 500;
int position;

int proportional;
int derivative;
int integral;
int lastError;

int maxSpeed = 230;
int minSpeed = 190;
int speed = 210;

float kp = 0.20;
float ki = 0.10;
float kd = 0.25;
float pid;
float pidRight;
float pidLeft;

void PID() {

  position = Read_Right_Sensor();

  proportional = position - setPoint;
  derivative = proportional - lastError;

  pid = (proportional * kp) + (derivative * kd);

  lastError = proportional;

  pidRight = speed + pid - 10;
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
    motor.MoveForwards(pidRight , pidLeft);
  }
}


bool flag_turn_right = false;
bool flag_turn_left  = false;
bool flag_turn_u     = false;
bool flag_PID = false;
int KEEP_FORWATDS_TIME_START  = 250;
int KEEP_FORWARDS_TIME_FINISH = 500;
int TURN_TIME   =  250;
int TURN_U_TIME =  450;


enum Action {
    PREPARATION,
    FORWARDS_START,
    TURN,
    FORWARDS_FINISH
};

static Action turn_stage = PREPARATION;
static unsigned long action_start = 0;


void TurnLeft() {
  
  unsigned long time = millis();

  if (turn_stage == PREPARATION) {
    Serial.println ("PREPARATION");
    action_start = time;
    turn_stage = FORWARDS_START;
  }

  if (turn_stage == FORWARDS_START) {
    Serial.println ("FORWARDS_START");
    motor.MoveForwards(MAX_SPEED, MAX_SPEED);
    if (time - action_start >= KEEP_FORWATDS_TIME_START) {
      action_start = time;
      turn_stage = TURN;
    }
  }
  else if (turn_stage == TURN) {
    Serial.println ("TURN");
    motor.TurnLeft(BREAK, MAX_SPEED);
    if (time - action_start >= TURN_TIME) {
      action_start = time;
      turn_stage = FORWARDS_FINISH;
    }
  }
  else if (turn_stage == FORWARDS_FINISH) {
    Serial.println ("FORWARDS_FINISH");
    motor.MoveForwards(MAX_SPEED, MAX_SPEED);
    if (time - action_start >= KEEP_FORWARDS_TIME_FINISH) {
      turn_stage = PREPARATION;
      flag_turn_left = false;
      motor.MoveForwards (0 , 0);
      Serial.print ("WAITING");
    }
  }
}


void TurnRight() {

  unsigned long time = millis();

  if (turn_stage == PREPARATION) {
    Serial.println ("PREPARATION");
    action_start = time;
    turn_stage = FORWARDS_START;
  }

  if (turn_stage == FORWARDS_START) {
    Serial.println ("FORWARDS_START");
    motor.MoveForwards(MAX_SPEED, MAX_SPEED - 20);
    if (time - action_start >= KEEP_FORWATDS_TIME_START) {
      action_start = time;
      turn_stage = TURN;
    }
  }
  else if (turn_stage == TURN) {
    Serial.println ("TURN");
    motor.TurnRight(MAX_SPEED, BREAK);
    if (time - action_start >= TURN_TIME + TURN_TIME) {
      action_start = time;
      turn_stage = FORWARDS_FINISH;
    }
  }
  else if (turn_stage == FORWARDS_FINISH) {
    Serial.println ("FORWARDS_FINISH");
    motor.MoveForwards(MAX_SPEED, MAX_SPEED - 20);
    
    if (time - action_start >= KEEP_FORWARDS_TIME_FINISH - 100) {
      turn_stage = PREPARATION;
      flag_turn_right = false;
      motor.MoveForwards (0 , 0);
      Serial.print ("WAITING");
    }
  }
}


enum ActionU {
  PREPARATION_U,
  TURN_U,
};
static ActionU turn_stage_u = PREPARATION_U;
static unsigned long action_start_u = 0;

void TurnU() {

  unsigned long time_u = millis();

  if (turn_stage_u == PREPARATION_U) {
    Serial.println ("PREPARATION");
    action_start_u = time_u;
    turn_stage_u = TURN_U;
  }

  if (turn_stage_u == TURN_U) {
    Serial.println ("TURN U");
    motor.TurnAroundLeft (MAX_SPEED, MAX_SPEED);
    
    if (time_u - action_start_u >= TURN_U_TIME) {
      turn_stage_u = PREPARATION_U;
      flag_turn_u = false;
      motor.MoveForwards (0 , 0);
      Serial.print ("WAITING");
    }
  }
}


bool flag_RobotIsTurning = false;
int SpeedLeft = 255;
int SpeedRight = 255;

void loop() {

  Bluetooth();

  /*if (dato_BT == 'q') {
    SpeedLeft += 10;
  }
  else if (dato_BT == 'a') {
    SpeedLeft -= 10;
  }
  else if (dato_BT == 'w') {
    SpeedRight += 10;
  }
  else if (dato_BT == 's') {
    SpeedRight -= 10;
  }
  else if (dato_BT == 'z') {
    SerialBT.print ("LEFT = ");
    SerialBT.println (SpeedLeft);
    SerialBT.print ("RIGHT = ");
    SerialBT.println (SpeedRight);
  }



  if (button_order (PIN_START)) {
    motor.MoveForwards (SpeedLeft , SpeedRight);
  }
  if (button_order (PIN_LEFT)) {
    motor.MoveForwards (0 , 0);
  }*/
  
  if (dato_BT == 'q') {
    kp += 0.05;
  }
  else if (dato_BT == 'a') {
    kp -= 0.05;
  }
  else if (dato_BT == 'w') {
    kd += 0.05;
  }
  else if (dato_BT == 's') {
    kd -= 0.05;
  }
  else if (dato_BT == 'e') {
   KEEP_FORWATDS_TIME_START += 50;
  }
  else if (dato_BT == 'd') {
   KEEP_FORWATDS_TIME_START -= 50;
  }
  else if (dato_BT == 'r') {
   TURN_TIME += 50;
  }
  else if (dato_BT == 'f') {
   TURN_TIME -= 50;
  }
  else if (dato_BT == 't') {
   KEEP_FORWARDS_TIME_FINISH += 50;
  }
  else if (dato_BT == 'g') {
   KEEP_FORWARDS_TIME_FINISH -= 50;
  }
  else if (dato_BT == 'y') {
   TURN_U_TIME += 50;
  }
  else if (dato_BT == 'h') {
   TURN_U_TIME -= 50;
  }
  else if (dato_BT == 'z') {
    SerialBT.print ("KP = ");
    SerialBT.println (kp);
    SerialBT.print ("KD = ");
    SerialBT.println (kd);
    SerialBT.println (" ");
    SerialBT.print ("KEEP START = ");
    SerialBT.println (KEEP_FORWATDS_TIME_START);
    SerialBT.print ("TURN = ");
    SerialBT.println (TURN_TIME);
    SerialBT.print ("KEEP FINISH = ");
    SerialBT.println (KEEP_FORWARDS_TIME_FINISH);
    SerialBT.print ("TURN U = ");
    SerialBT.println (TURN_U_TIME);
  }

  int front = Read_Front_Sensor();
  int left  = Read_Left_Sensor ();
  int right = Read_Right_Sensor();

  if (!flag_RobotIsTurning) {

    if (wall_detected(right)) {

      if (wall_detected (front)) {

        if (wall_detected (left)) {
          //Turn U
          SerialBT.println ("Turn U");
          flag_turn_u = true;
          flag_RobotIsTurning = true;
        }
        else {
          //Turn Left
          SerialBT.println ("Turn Left");
          flag_turn_left = true;
          flag_RobotIsTurning = true;
        }
      }

      else {
        SerialBT.println ("Forwards");
        motor.MoveForwards (MAX_SPEED , 225);
      }
    }
    
    else {
      //turn Right
      SerialBT.println ("Turn Right");
      flag_turn_right = true;
      flag_RobotIsTurning = true;
    }
  }

  
  if (flag_turn_right) {
    TurnRight();
    if (!flag_turn_right) {
      flag_RobotIsTurning = false;
    }
  }
  
  if (flag_turn_left) {
    TurnLeft();
    if (!flag_turn_left) {
      flag_RobotIsTurning = false;
    }
  }

  if (flag_turn_u) {
    TurnU();
    if (!flag_turn_u) {
      flag_RobotIsTurning = false;
    }
  }
}

/*if (button_order (PIN_START)) {
    flag_PID = true;
  }
  else if (button_order (PIN_LEFT)) {
    flag_PID = false;
  }


  if (flag_PID) {
    PID();
  }
  else if (!flag_PID) {
    motor.MoveForwards (0 , 0);
  }




  if (dato_BT == 'q') {
    kp += 0.05;
  }
  else if (dato_BT == 'a') {
    kp -= 0.05;
  }
  else if (dato_BT == 'w') {
    kd += 0.05;
  }
  else if (dato_BT == 's') {
    kd -= 0.05;
  }
  else if (dato_BT == 'z') {
    SerialBT.print ("KP = ");
    SerialBT.println (kp);
    SerialBT.print ("KD = ");
    SerialBT.println (kd);
  }*/