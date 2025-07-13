#ifndef MOTOR_H
#define MOTOR_H

class Motor {
    private:

        int firstPin , secondPin;
        int pwmPin;
        int channel ;
        int pwmFrequency;
        int pwmResolution;

    public:

        Motor (int firstPin , int secondPin , int pwmPin , int channel , 
               int frequency , int resolution);

        void MoveForwards (int speed);
        void MoveBackwards (int speed);
        void Brake ();
        void StayStill ();
};

class MotorPair {
    private:

        Motor motorLeft , motorRight;

    public:
        
        MotorPair (int pin_Mr1 , int pin_Mr2 , int pwm_Mr , int channel_Mr ,
                   int pin_Ml1 , int pin_Ml2 , int pwm_Ml , int channel_Ml , 
                   int frequency , int resolution);

        void MoveForwards  (int speed_Left , int speed_Right);
        void MoveBackwards (int speed_Left , int speed_Right);
        void TurnLeft  (int speed_Left , int speed_Right);
        void TurnRight (int speed_Left , int speed_Right);
        void Brake ();
        void StayStill ();   
};

#endif