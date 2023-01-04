#pragma once

#include <frc/XboxController.h>
#include <frc2/command/button/JoystickButton.h>

#include "utils/ports/CONTROL_PORTS.h"

class Controlboard {
   public:
    Controlboard();
    double GetDriveX();
    double GetDriveY();
    double GetRotX();
    double GetRotY();

    bool GetAButton();

    frc::XboxController _xboxDrive{DRIVER_XBOX_USB_PORT};

    frc2::JoystickButton _aButton{&_xboxDrive, frc::XboxController::Button::kA};
    frc2::JoystickButton _bButton{&_xboxDrive, frc::XboxController::Button::kB};
    frc2::JoystickButton _yButton{&_xboxDrive, frc::XboxController::Button::kY};
    frc2::JoystickButton _xButton{&_xboxDrive, frc::XboxController::Button::kX};
    frc2::JoystickButton _startButton{&_xboxDrive, frc::XboxController::Button::kStart};
    frc2::JoystickButton _backButton{&_xboxDrive, frc::XboxController::Button::kBack};
    frc2::JoystickButton _leftBumper{&_xboxDrive, frc::XboxController::Button::kLeftBumper};
    frc2::JoystickButton _rightBumper{&_xboxDrive, frc::XboxController::Button::kRightBumper};
};
