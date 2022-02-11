#include "Manual_fast.h"
#include "variables.h"
#include "common.h"

Manual_Fast Test;
Common_functions common;
void setup()
{
    //PWM
    pinMode(mot1, OUTPUT);
    pinMode(mot2, OUTPUT);
    pinMode(mot3, OUTPUT);
    //DIR
    pinMode(dir1, OUTPUT);
    pinMode(dir2, OUTPUT);
    pinMode(dir3, OUTPUT);

    Serial.begin(115200);
    Serial2.begin(115200);
    Serial3.begin(115200);
    yaw_new = 0;
    yaw = 0;
    lastP = 0;
    slowly = false;
}

void loop()
{
    yaw_old = yaw_new;

    if (Serial2.available() > 0)
    {
        yaw_new = Serial2.read();
    }
    if (Serial3.available() > 0)
    {
        bluetooth = Serial3.read();
    }

#ifdef DEBUG
    Serial.print("Blue: ");
    Serial.print(bluetooth);
#endif
    if ((yaw_new - yaw_old) > 0)
    {
        yaw = yaw + (yaw_new - yaw_old);
    }
    else if ((yaw_new - yaw_old) < 0)
    {
        yaw = yaw + (yaw_new - yaw_old);
    }
    lastP = error;
    error = yaw - setpoint;

#ifdef DEBUG
    Serial.print("  Yawnew: ");
    Serial.print(yaw_new);
    Serial.print("  Yaw: ");
    Serial.print(yaw);
    Serial.print("  Error: ");
    Serial.print(error);
#endif
    switch (bluetooth)
    {
    case 'F':
        Test.forward();
        break;
    case 'B':
        Test.backward();
        break;
    case 'R':
        Test.right();
        break;
    case 'L':
        Test.left();
        break;
    case 'C':
        common.rotate_clockwise();
        break;
    case 'A':
        common.rotate_anticlockwise();
        break;
    default:
        common.stop_wheel();
        break;
    }
}