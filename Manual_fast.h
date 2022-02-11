#include "variables.h"
#define DEBUG
class Manual_Fast
{
public:
    int PID(float Kp, float kd)
    {
        error_rate = error - lastP;
        value = Kp * error + kd * error_rate;
        return int(value);
    }
    void forward()
    {
        digitalWrite(dir1, LOW);
        digitalWrite(dir2, HIGH);
        digitalWrite(dir3, HIGH);

        if (error < 0)
        {
            pwmMotor1 = constrain((basePWM1 - PID(Kp1f, Kd1f)), pwm_min, pwm_max);
            pwmMotor3 = constrain((basePWM3 + PID(Kp3f, Kd3f)), pwm_min, pwm_max);
            pwmMotor2 = 0;
#ifdef DEBUG
            Serial.print("  Motor1's PWM: ");
            Serial.print(pwmMotor1);
            Serial.print("  Motor2's PWM: ");
            Serial.print(pwmMotor2);
            Serial.print("  Motor3's PWM: ");
            Serial.print(pwmMotor3);
            Serial.println();
#endif
            analogWrite(mot1, pwmMotor1);
            analogWrite(mot3, pwmMotor3);
            analogWrite(mot2, 0);
        }
        else if (error > 0)
        {

            pwmMotor1 = constrain((basePWM1 - PID(Kp1f, Kd1f)), pwm_min, pwm_max);
            pwmMotor3 = constrain((basePWM3 + PID(Kp3f, Kd3f)), pwm_min, pwm_max);
            pwmMotor2 = 0;

#ifdef DEBUG
            Serial.print("  Motor1's PWM: ");
            Serial.print(pwmMotor1);
            Serial.print("  Motor2's PWM: ");
            Serial.print(pwmMotor2);
            Serial.print("  Motor3's PWM: ");
            Serial.print(pwmMotor3);
            Serial.println();
#endif

            analogWrite(mot1, pwmMotor1);
            analogWrite(mot3, pwmMotor3);
            analogWrite(mot2, 0);
        }
        else
        {
            pwmMotor1 = basePWM1;
            pwmMotor3 = basePWM3;
            pwmMotor2 = 0;
#ifdef DEBUG
            Serial.print("  Motor1's PWM: ");
            Serial.print(pwmMotor1);
            Serial.print("  Motor2's PWM: ");
            Serial.print(pwmMotor2);
            Serial.print("  Motor3's PWM: ");
            Serial.print(pwmMotor3);
            Serial.println();
#endif
            analogWrite(mot1, pwmMotor1);
            analogWrite(mot3, pwmMotor3);
            analogWrite(mot2, pwmMotor2);
        }
    }
    /************************************************************************/
    /*********************************BACKWARD*******************************/

    void backward()
    {

        digitalWrite(dir1, HIGH);
        digitalWrite(dir2, LOW);
        digitalWrite(dir3, LOW);
        if (error > 0)
        {
            pwmMotor1 = constrain((basePWM1 + PID(Kp1b, Kd1b)), pwm_min, pwm_max);
            pwmMotor3 = constrain((basePWM3 - PID(Kp3b, Kd3b)), pwm_min, pwm_max);
            pwmMotor2 = 0;

#ifdef DEBUG
            Serial.print("  Motor1's PWM: ");
            Serial.print(pwmMotor1);
            Serial.print("  Motor2's PWM: ");
            Serial.print(pwmMotor2);
            Serial.print("  Motor3's PWM: ");
            Serial.print(pwmMotor3);
            Serial.println();
#endif

            analogWrite(mot1, pwmMotor1);
            analogWrite(mot3, pwmMotor3);
            analogWrite(mot2, 0);
        }
        else if (error < 0)
        {
            pwmMotor1 = constrain((basePWM1 + PID(Kp1b, Kd1b)), pwm_min, pwm_max);
            pwmMotor3 = constrain((basePWM3 - PID(Kp3b, Kd3b)), pwm_min, pwm_max);
            pwmMotor2 = 0;
#ifdef DEBUG
            Serial.print("  Motor1's PWM: ");
            Serial.print(pwmMotor1);
            Serial.print("  Motor2's PWM: ");
            Serial.print(pwmMotor2);
            Serial.print("  Motor3's PWM: ");
            Serial.print(pwmMotor3);
            Serial.println();
#endif

            analogWrite(mot1, pwmMotor1);
            analogWrite(mot3, pwmMotor3);
            analogWrite(mot2, 0);
        }
        else
        {
            pwmMotor1 = basePWM1;
            pwmMotor3 = basePWM3;
            pwmMotor2 = 0;
#ifdef DEBUG
            Serial.print("  Motor1's PWM: ");
            Serial.print(pwmMotor1);
            Serial.print("  Motor2's PWM: ");
            Serial.print(pwmMotor2);
            Serial.print("  Motor3's PWM: ");
            Serial.print(pwmMotor3);
            Serial.println();
#endif

            analogWrite(mot1, pwmMotor1);
            analogWrite(mot3, pwmMotor3);
            analogWrite(mot2, pwmMotor2);
        }
    }
    /************************************************************************/
    /******************************RIGHT*****************************/
    void right()
    {
        digitalWrite(dir1, LOW);
        digitalWrite(dir2, LOW);
        digitalWrite(dir3, LOW);

        if (error > 0)
        {

            pwmMotor1 = constrain((basePWM_side - PID(Kp1r, Kd1r)), pwm_min_side, pwm_max_side);
            pwmMotor3 = constrain((basePWM_side - PID(Kp3r, Kd3r)), pwm_min_side, pwm_max_side);
            pwmMotor2 = constrain((basePWM + 20 + PID(Kp2r, Kd2r)), pwm_min, pwm_max);

#ifdef DEBUG
            Serial.print("  Motor1's PWM: ");
            Serial.print(pwmMotor1);
            Serial.print("  Motor2's PWM: ");
            Serial.print(pwmMotor2);
            Serial.print("  Motor3's PWM: ");
            Serial.print(pwmMotor3);
            Serial.println();
#endif

            analogWrite(mot1, pwmMotor1);
            analogWrite(mot3, pwmMotor3);
            analogWrite(mot2, pwmMotor2);
        }
        else if (error < 0)
        {

            pwmMotor1 = constrain((basePWM_side - PID(Kp1r, Kd1r)), pwm_min_side, pwm_max_side);
            pwmMotor3 = constrain((basePWM_side - PID(Kp3r, Kd3r)), pwm_min_side, pwm_max_side);
            pwmMotor2 = constrain((basePWM + 20 + PID(Kp2r, Kd2r)), pwm_min, pwm_max);

#ifdef DEBUG
            Serial.print("  Motor1's PWM: ");
            Serial.print(pwmMotor1);
            Serial.print("  Motor2's PWM: ");
            Serial.print(pwmMotor2);
            Serial.print("  Motor3's PWM: ");
            Serial.print(pwmMotor3);
            Serial.println();
#endif

            analogWrite(mot1, pwmMotor1);
            analogWrite(mot3, pwmMotor3);
            analogWrite(mot2, pwmMotor2);
        }
        else
        {
            pwmMotor1 = constrain((basePWM_side), pwm_min_side, pwm_max_side);
            pwmMotor3 = constrain((basePWM_side), pwm_min_side, pwm_max_side);
            pwmMotor2 = constrain((basePWM + 20), pwm_min, pwm_max);

#ifdef DEBUG
            Serial.print("  Motor1's PWM: ");
            Serial.print(pwmMotor1);
            Serial.print("  Motor2's PWM: ");
            Serial.print(pwmMotor2);
            Serial.print("  Motor3's PWM: ");
            Serial.print(pwmMotor3);
            Serial.println();
#endif

            analogWrite(mot1, pwmMotor1);
            analogWrite(mot3, pwmMotor3);
            analogWrite(mot2, pwmMotor3);
        }
    }

    /************************************************************************/
    /***********************************LEFT*********************************/
    void left()
    {
        digitalWrite(dir1, HIGH);
        digitalWrite(dir2, HIGH);
        digitalWrite(dir3, HIGH);

        if (error > 0)
        {

            pwmMotor1 = constrain((basePWM_side + PID(Kp1l, Kd1l)), pwm_min_side, pwm_max_side);
            pwmMotor3 = constrain((basePWM_side + PID(Kp3l, Kd3l)), pwm_min_side, pwm_max_side);
            pwmMotor2 = constrain((basePWM + 20 - PID(Kp2l, Kd2l)), pwm_min, pwm_max);

#ifdef DEBUG
            Serial.print("  Motor1's PWM: ");
            Serial.print(pwmMotor1);
            Serial.print("  Motor2's PWM: ");
            Serial.print(pwmMotor2);
            Serial.print("  Motor3's PWM: ");
            Serial.print(pwmMotor3);
            Serial.println();
#endif

            analogWrite(mot1, pwmMotor1);
            analogWrite(mot3, pwmMotor3);
            analogWrite(mot2, pwmMotor2);
        }
        else if (error < 0)
        {

            pwmMotor1 = constrain((basePWM_side + PID(Kp1l, Kd1l)), pwm_min_side, pwm_max_side);
            pwmMotor3 = constrain((basePWM_side + PID(Kp3l, Kd3l)), pwm_min_side, pwm_max_side);
            pwmMotor2 = constrain((basePWM + 20 - PID(Kp2l, Kd2l)), pwm_min, pwm_max);

#ifdef DEBUG
            Serial.print("  Motor1's PWM: ");
            Serial.print(pwmMotor1);
            Serial.print("  Motor2's PWM: ");
            Serial.print(pwmMotor2);
            Serial.print("  Motor3's PWM: ");
            Serial.print(pwmMotor3);
            Serial.println();
#endif

            analogWrite(mot1, pwmMotor1);
            analogWrite(mot3, pwmMotor3);
            analogWrite(mot2, pwmMotor2);
        }
        else
        {
            pwmMotor1 = constrain((basePWM_side), pwm_min_side, pwm_max_side);
            pwmMotor3 = constrain((basePWM_side), pwm_min_side, pwm_max_side);
            pwmMotor2 = constrain((basePWM + 20), pwm_min, pwm_max);

#ifdef DEBUG
            Serial.print("  Motor1's PWM: ");
            Serial.print(pwmMotor1);
            Serial.print("  Motor2's PWM: ");
            Serial.print(pwmMotor2);
            Serial.print("  Motor3's PWM: ");
            Serial.print(pwmMotor3);
            Serial.println();
#endif

            analogWrite(mot1, pwmMotor1);
            analogWrite(mot3, pwmMotor3);
            analogWrite(mot2, pwmMotor2);
        }
    }
};