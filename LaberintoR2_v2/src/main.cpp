#include <Arduino.h>
#include <Wire.h>
#include <BluetoothSerial.h>

//Sensores de Ultrasonido
#define PIN_ECHO_DER        39 
#define PIN_ECHO_IZQ        35 
#define PIN_ECHO_FRONT      34 
#define PIN_TRIGGER_DER     32 
#define PIN_TRIGGER_IZQ     33 
#define PIN_TRIGGER_FRONT   25 
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

//Funciones de pulsadores
int state_start;
int state_right;
int state_left;
int casos_pulsadores = 'E';
bool start;
bool right;
bool left;
bool start_right;
bool start_left;
bool start_bool;
unsigned long start_time = 0;


//Funciones Globales de Ultrasonidos
long distancia_der;
long distancia_izq;
long distancia_front;
int sensores_por_derecha;



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
#define RAD_A_DEG = 57.295779
 
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
  pinMode (PIN_ECHO_DER   , INPUT);
  pinMode (PIN_ECHO_IZQ   , INPUT);
  pinMode (PIN_ECHO_FRONT , INPUT);
  pinMode (PIN_TRIGGER_DER   , OUTPUT);
  pinMode (PIN_TRIGGER_IZQ   , OUTPUT);
  pinMode (PIN_TRIGGER_FRONT , OUTPUT);

  digitalWrite (PIN_TRIGGER_DER , LOW);
  digitalWrite (PIN_TRIGGER_IZQ , LOW);
  digitalWrite (PIN_TRIGGER_FRONT , LOW);

  //Pulsadores
  pinMode (PIN_START , INPUT_PULLUP);
  pinMode (PIN_RIGHT , INPUT_PULLUP);
  pinMode (PIN_LEFT  , INPUT_PULLUP);
  start = false;
  right = false;
  left  = false;
  start_bool = false;
  casos_pulsadores = 'E';

  //Drivers de Motor
  pinMode (PIN_A1 , OUTPUT);
  pinMode (PIN_A2 , OUTPUT);
  pinMode (PIN_B1 , OUTPUT);
  pinMode (PIN_B2 , OUTPUT);
  pinMode (PIN_PWM_A , OUTPUT);
  pinMode (PIN_PWM_B , OUTPUT);

  //MPU6050
  Wire.begin(21,22); // D2(GPIO4)=SDA / D1(GPIO5)=SCL
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(115200);

  //Bluetooth
  Serial.begin(115200);
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

void Pulsadores() {
  state_right = digitalRead (PIN_RIGHT);
  if (state_right == LOW) {
    right = true;
  } else {!right;}
  
  state_left = digitalRead (PIN_LEFT);
  if (state_left == LOW) {
    left = true;
  } else {!left;}

  if (right == true) {casos_pulsadores = 'R';}
  else if (left == true) {casos_pulsadores = 'L';}
  if (!right && !left) {casos_pulsadores = 'E';}


  if (right || left) {
    state_start = digitalRead (PIN_START);
    if (state_start == LOW) {
      start = true;
      start_time = millis(); 
    } else {!start;}
  }
  int tiempo_espera_start = 5000;
  
  if (start && millis() < start_time + tiempo_espera_start) {
    start_bool = false;
  } else {
    start_bool = true;
  }

  if (start && start_bool) {
    switch (casos_pulsadores) {

      case 'R': {
        start_right = true;
        break;
      }

      case 'L': {
        start_left = true;
        break;
      }

      case 'E': {
      Serial.print ("Esperando");
      break;
      }
    }
  }
  delay (250);
}

void SensoresMedicion() {
  long tiempo_der;
  long tiempo_izq;
  long tiempo_front;

  digitalWrite (PIN_TRIGGER_DER , HIGH);
  delayMicroseconds(10);
  digitalWrite (PIN_TRIGGER_DER , LOW);
  tiempo_der = pulseIn (PIN_ECHO_DER , HIGH);
  distancia_der = tiempo_der / 59; 

  digitalWrite (PIN_TRIGGER_IZQ , HIGH);
  delayMicroseconds(10);
  digitalWrite (PIN_TRIGGER_IZQ , LOW);
  tiempo_izq = pulseIn (PIN_ECHO_IZQ , HIGH);
  distancia_izq = tiempo_izq / 59;

  digitalWrite (PIN_TRIGGER_FRONT , HIGH);
  delayMicroseconds(10);
  digitalWrite (PIN_TRIGGER_FRONT , LOW);
  tiempo_front = pulseIn (PIN_ECHO_FRONT , HIGH);
  distancia_front = tiempo_front / 59;
}

void SensoresPorDerecha() {
  
}


void loop() {

  Bluetooth();
  Pulsadores();
  
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

   dt = (millis() - tiempo_prev) / 1000.0;
   tiempo_prev = millis();
 
   //Aplicar el Filtro Complementario
   Angle[0] = 0.98 *(Angle[0]+Gy[0]*dt) + 0.02*Acc[0];
   Angle[1] = 0.98 *(Angle[1]+Gy[1]*dt) + 0.02*Acc[1];

   //Integración respecto del tiempo paras calcular el YAW
   Angle[2] = Angle[2]+Gy[2]*dt;
   int yaw;
   yaw = Angle[2];
 
   //Mostrar los valores por consola
   valores = "90, " +String(Angle[0]) + "," + String(Angle[1]) + "," + String(Angle[2]) + ", -90"; //Angle2 YAW IZQ -90 DER 90
   Serial.println(valores);
   delay(10);
}

