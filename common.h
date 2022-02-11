#include "variables.h"
#define DEBUG
class Common_functions
{
    public:
    /************************************************************************/
    /******************************ROTATE_CLOCKWISE*****************************/
    void rotate_clockwise()
    {

        pwmMotor1 = 50;
        pwmMotor2 = 50;
        pwmMotor3 = 50;

#ifdef DEBUG
        Serial.print("  Motor1's PWM: ");
        Serial.print(pwmMotor1);
        Serial.print("  Motor2's PWM: ");
        Serial.print(pwmMotor2);
        Serial.print("  Motor3's PWM: ");
        Serial.print(pwmMotor3);
        Serial.println();
#endif
        digitalWrite(dir1, HIGH);
        digitalWrite(dir2, LOW);
        digitalWrite(dir3, HIGH);

        analogWrite(mot1, pwmMotor1);
        analogWrite(mot3, pwmMotor3);
        analogWrite(mot2, pwmMotor2);
    }
    /************************************************************************/
    /******************************ROTATE_ANTICLOCKWISE*****************************/
    void rotate_anticlockwise()
    {

        pwmMotor1 = 50;
        pwmMotor2 = 50;
        pwmMotor3 = 50;

#ifdef DEBUG
        Serial.print("  Motor1's PWM: ");
        Serial.print(pwmMotor1);
        Serial.print("  Motor2's PWM: ");
        Serial.print(pwmMotor2);
        Serial.print("  Motor3's PWM: ");
        Serial.print(pwmMotor3);
        Serial.println();
#endif

        digitalWrite(dir1, LOW);
        digitalWrite(dir2, HIGH);
        digitalWrite(dir3, LOW);

        analogWrite(mot1, pwmMotor1);
        analogWrite(mot3, pwmMotor3);
        analogWrite(mot2, pwmMotor2);
    }
    /************************************************************************/
    /******************************STOPED*****************************/
    void stoped()
    {
        yaw = 0;
        timer = millis();
#ifdef DEBUG
        Serial.print("  Motor1's PWM: ");
        Serial.print(0);
        Serial.print("  Motor2's PWM: ");
        Serial.print(0);
        Serial.print("  Motor3's PWM: ");
        Serial.print(0);
        Serial.println();
#endif
        analogWrite(mot1, 0);
        analogWrite(mot3, 0);
        analogWrite(mot2, 0);
    }
    /************************************************************************/
    /******************************STOPED*****************************/
    void stop_wheel()
    {
        yaw = 0;
        timer = millis();
#ifdef DEBUG
        Serial.print("  Motor1's PWM: ");
        Serial.print(0);
        Serial.print("  Motor2's PWM: ");
        Serial.print(0);
        Serial.print("  Motor3's PWM: ");
        Serial.print(0);
        Serial.println();
#endif
        analogWrite(mot1, 0);
        analogWrite(mot3, 0);
        analogWrite(mot2, 0);
    }
};