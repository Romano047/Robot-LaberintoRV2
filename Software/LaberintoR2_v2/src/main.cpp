#include <Arduino.h>
#include <Wire.h>
#include <BluetoothSerial.h>
#include <motors_TB6612fng.h>

//Sensores de Ultrasonido
#define PIN_ECHO_R        39 
#define PIN_ECHO_L        35 
#define PIN_ECHO_FRONT    34 
#define PIN_TRIGGER_R     32 
#define PIN_TRIGGER_L     33 
#define PIN_TRIGGER_FRONT 25 
//Pulsadores
#define PIN_START 17
#define PIN_RIGHT 18
#define PIN_LEFT  19
//Drivers de Motor
#define PIN_A1    26
#define PIN_A2    27
#define PIN_B1     2
#define PIN_B2    13
#define PIN_PWM_A  4
#define PIN_PWM_B 16

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


void setup() {
  Serial.begin (115200);

  //Sensores de Ultrasonido
  pinMode (PIN_ECHO_R   , INPUT);
  pinMode (PIN_ECHO_L   , INPUT);
  pinMode (PIN_ECHO_FRONT , INPUT);
  pinMode (PIN_TRIGGER_R   , OUTPUT);
  pinMode (PIN_TRIGGER_L   , OUTPUT);
  pinMode (PIN_TRIGGER_FRONT , OUTPUT);

  digitalWrite (PIN_TRIGGER_R , LOW);
  digitalWrite (PIN_TRIGGER_L , LOW);
  digitalWrite (PIN_TRIGGER_FRONT , LOW);

  //MPU6050
  Wire.begin(21,22);
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  tiempo_prev = millis();

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

#define wallDetected 1000

bool wall_detected (int sensor) {
  if (sensor < wallDetected) {
    return true;
  } else {
    return false;
  }
}

/*
  digitalWrite (PIN_TRIGGER_DER , HIGH);
  delayMicroseconds(10);
  digitalWrite (PIN_TRIGGER_DER , LOW);
  tiempo_der = pulseIn (PIN_ECHO_DER , HIGH);
  distancia_der = tiempo_der / 59; */


void Read_MPU6050 () {

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
   delay(10);*/
}





#define MAX_SPEED 255
#define MID_SPEED 100
#define LOW_SPEED 50

float actualYaw = 0;
double Turnning = false;
float objetiveYawRight;

void loop() {

  /*#define MAX_SPEED 255
  #define MID_SPEED 100
  #define LOW_SPEED 50*/

  int front = Read_Front_Sensor();
  int left  = Read_Left_Sensor ();
  int right = Read_Right_Sensor();

  Read_MPU6050();

  Serial.println (Angle[2]);

  if (wall_detected (right) && !Turnning) {
    Serial.println ("Adelante");
  }
  else if (!wall_detected (right)) {
    Turnning = true;
    actualYaw = Angle[2];
    objetiveYawRight = actualYaw - 90;
  }


  if (Turnning) {

    if (Angle[2] > objetiveYawRight) {
      Serial.println ("Doblando");
    }
    Turnning = false;
  }
}

