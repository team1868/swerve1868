#include "subsystems/Controlboard.h"

#include "utils/consts/CONTROL_CONSTS.h"

Controlboard::Controlboard() {}

// Left is positive X in terms of field, negative so our controller aligns
double Controlboard::GetDriveX() { return -_xboxDrive.GetLeftY(); }

// Forward is positive Y in terms of field, negative so our controller aligns
double Controlboard::GetDriveY() { return -_xboxDrive.GetLeftX(); }

// Needs to align with our rotation, counterclockwise (left) must be positive to align
double Controlboard::GetRotX() { return -_xboxDrive.GetRightX(); }

// Unused at the moment
double Controlboard::GetRotY() { return _xboxDrive.GetRightY(); }

bool Controlboard::GetAButton() { return _xboxDrive.GetAButton(); }
