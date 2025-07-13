#include "Arduino.h"
#include "motors_TB6612fng.h"

Motor::Motor (int pin1 , int pin2 , int pwmPin , int channel , int frequency , int resolution)
    :        firstPin (pin1) , secondPin (pin2) , channel (channel) , pwmPin (pwmPin) , 
             pwmFrequency (frequency) , pwmResolution (resolution) {

    pinMode (firstPin  , OUTPUT);
    pinMode (secondPin , OUTPUT);
    pinMode (pwmPin    , OUTPUT);

    ledcSetup(channel , pwmFrequency , pwmResolution);
    
    ledcAttachPin(pwmPin , channel);
}


void Motor::MoveForwards (int speed) {
    digitalWrite (firstPin  , HIGH);
    digitalWrite (secondPin , LOW);
    ledcWrite (channel , speed);
}

void Motor::MoveBackwards (int speed) {
    digitalWrite (firstPin  , LOW);
    digitalWrite (secondPin , HIGH);
    ledcWrite (channel , speed);
}

void Motor::Brake () {
    digitalWrite (firstPin  , HIGH);
    digitalWrite (secondPin , HIGH);
    ledcWrite (channel , 255);
}

void Motor::StayStill () {
    digitalWrite (firstPin  , LOW);
    digitalWrite (secondPin , LOW);
    ledcWrite (channel , 0);
}



MotorPair::MotorPair(int pinMr1, int pinMr2, int pwmMr , int channelMr,
                     int pinMl1, int pinMl2, int pwmMl , int channelMl, 
                     int frequency, int resolution)
    : motorRight(pinMr1, pinMr2, pwmMr, channelMr, frequency, resolution),
      motorLeft (pinMl1, pinMl2, pwmMl, channelMl, frequency, resolution) {
}


void MotorPair::MoveForwards (int speedLeft , int speedRight) {
    motorLeft.MoveForwards  (speedLeft);
    motorRight.MoveForwards (speedRight);
}

void MotorPair::MoveBackwards (int speedLeft , int speedRight) {
    motorLeft.MoveBackwards  (speedLeft);
    motorRight.MoveBackwards (speedRight);
}

void MotorPair::TurnLeft (int speedLeft , int speedRight) {
    motorLeft.MoveForwards  (speedLeft);
    motorRight.MoveForwards (speedRight);
}

void MotorPair::TurnRight (int speedLeft , int speedRight) {
    motorLeft.MoveForwards  (speedLeft);
    motorRight.MoveForwards (speedRight);
}

void MotorPair::Brake () {
    motorLeft.Brake  ();
    motorRight.Brake ();
}

void MotorPair::StayStill () {
    motorLeft.StayStill  ();
    motorRight.StayStill ();
}