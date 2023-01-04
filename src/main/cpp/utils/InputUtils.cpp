#include "utils/InputUtils.h"

#include "utils/consts/CONTROL_CONSTS.h"
#include "utils/consts/DRIVETRAIN_CONSTS.h"

meters_per_second_t ScaleJoystickX(double rawAxis)
{
    auto translationX = fabs(rawAxis) < STICK_DEADBAND ? 0 : pow(rawAxis, 3);
    return translationX * MAX_SPEED;
}

meters_per_second_t ScaleJoystickY(double rawAxis)
{
    auto translationY = fabs(rawAxis) < STICK_DEADBAND ? 0 : pow(rawAxis, 3);
    return translationY * MAX_SPEED;
}

degrees_per_second_t ScaleJoystickTheta(double rawAxis)
{
    auto angular = fabs(rawAxis) < STICK_DEADBAND ? 0 : pow(rawAxis, 3);
    return angular * MAX_ANGULAR_VELOCITY;
}
