#include <Arduino.h>
#include <PID_v1.h>
#include <ESP32Encoder.h> 
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include "robotconfig.h"
//#include "imuconfig.h"

//motor driver pin
const int EL_right = 26; //Enable Motor Driver Right
const int ZF_right = 27; 
const int VR_right = 25; //PWM Motor Driver Right
const int EL_left = 12; //Enable Motor Driver Left
const int ZF_left = 14; 
const int VR_left = 13; //PWM Motor Driver Left

//tachometer or signal pin
const int encoder_right = 15; 
const int encoder_left = 2;

//assign pin
const int MOTOR_PINS[6] = {EL_right, ZF_right, VR_right, EL_left, ZF_left, VR_left};
const int ENCODER_PINS[2] = {encoder_right, encoder_left};

int countLeft = 0;
int countRight = 0;



float calculateSpeed(long count, long time){
  float speed = (((count/encoder_cpr)/wheel_ratio) * PI  * wheel_dia) * (1000 / time); //Calculate speed
  return speed;
}
float tempspeed = calculateSpeed(0.5,10);

float SpeedtoPWM(float speed, int motor);
float SpeedtoPWM(float speed, int motor, float correction);

void left_wheel_tick() {
  if (digitalRead(ZF_left)==1) {
     countLeft++;
    }
    else {
     countLeft--;
    }
}

void right_wheel_tick() {
  if (digitalRead(ZF_right)==0){
    countRight++;
    }
    else {
    countRight--;
    }
}

int getLeftTick() {
    return countLeft;
}

int getRightTick() {
    return countRight;
}

int last_millis; //Number of tasks
bool ZF_value=false;
void setup() {
  pinMode(EL_left, OUTPUT);
  pinMode(EL_right, OUTPUT);
  pinMode(VR_left, OUTPUT);
  pinMode(VR_right, OUTPUT);
  pinMode(ZF_left, OUTPUT);
  pinMode(ZF_right, OUTPUT);

  pinMode(encoder_left, INPUT_PULLUP);
  pinMode(encoder_right, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoder_left), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder_right), right_wheel_tick, RISING);

  Serial.begin(9600);

  //initial condition
  digitalWrite(EL_left, HIGH);
  digitalWrite(EL_right, HIGH);
  digitalWrite(VR_left, HIGH);
  digitalWrite(VR_right, HIGH);
  digitalWrite(ZF_left, ZF_value);
  digitalWrite(ZF_right, !ZF_value);
}

void loop() {
  // put your main code here, to run repeatedly:
  // getLeftTick();
  // getRightTick();

  if(Serial.available()){
    int _ZF_value = Serial.read();
    switch (_ZF_value) {
      case '0':
        ZF_value = false;
        break;
      
      case '1':
        ZF_value = true;
        break;
      
      default:
        break;
    }

    Serial.print("ZF input: ");
    Serial.println(ZF_value);
    digitalWrite(ZF_left, !ZF_value); //nilai ZF kiri harus berkebalikan dengan kanan
    digitalWrite(ZF_right, ZF_value);
    Serial.print("ZF left: ");
    Serial.println(digitalRead(ZF_left)); 
    Serial.print("ZF right: ");
    Serial.println(digitalRead(ZF_right));
    Serial.println("-------");
    Serial.print("Speed =");
    Serial.println(tempspeed);
  }

  if(millis() - last_millis > 1000){
    last_millis = millis();
    Serial.print("Left: ");
    Serial.println(getLeftTick());
    Serial.print("Right: ");
    Serial.println(getRightTick());
  }
}