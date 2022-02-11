#ifndef _variables_H_
#define _variables_H_

char bluetooth= 'S';

int mot1 = 5;
int mot2 = 6;
int mot3 = 4;
int dir1 = 26;
int dir2 = 28;
int dir3 = 24;
int pwmMotor1;
int pwmMotor2;
int pwmMotor3;
int basePWM = 110;
int basePWM1 = 120;
int basePWM2 = 140;
int basePWM3 = 140;
int basePWM_side = 70;
int pwm_min= 100;
int pwm_max = 180;
int pwm_min_side= 65;
int pwm_max_side = 95;
int yaw;
int temp;
int count;
int slowly;
int8_t yaw_old;
int8_t yaw_new;
int setpoint = 0;
int error = 0;
int add_pid;
int error_rate;
float value;
float Kp1f = 5;
float Kp2f = 0;
float Kp3f = 10;
float Kp1b = 1.5;
float Kp2b = 1.5;
float Kp3b = 1.5;
float lastP;
float Kp1r = 2;
float Kp2r = 3;
float Kp3r = 2;
float Kp1l = 2;
float Kp2l = 3;
float Kp3l = 2;

float Kd1f = 18;
float Kd2f = 5;
float Kd3f = 15;
float Kd1b = 5;
float Kd2b = 5;
float Kd3b = 5;
float Kd1r = 5;
float Kd2r = 5;
float Kd3r = 5;
float Kd1l = 5;
float Kd2l = 5;
float Kd3l = 5;

unsigned long timer = 0;
#endif