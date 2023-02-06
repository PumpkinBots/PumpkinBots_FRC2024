// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/XboxController.h>
#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"


class GrabberSubsystem {
public:
    GrabberSubsystem(
        TalonSRX& m_GrabberIntake,
        TalonSRX& m_GrabberAngle,
        frc::XboxController& stick
    );

    // Initialize the subsystem from Robot::RobotInit().
    void RobotInit();
    // Call this when entering any mode.
    void ModeInit();
    // Call this in Periodic() function to check button and set motor.
    bool RunPeriodic();
    // Call this in Autonomous Periodic() function.
    // enabled: run the drive if true else stop.
    bool RunAutonomous();

    // Stop the motor and disable run state.
    void StopMotor();

public:

private:
    bool m_runGrabberIntake = false;
    // Non-owning reference to the motor controller.
    TalonSRX& m_GrabberIntake;
    // Non-owning reference to the joystick.
    frc::XboxController& m_xbox;

};