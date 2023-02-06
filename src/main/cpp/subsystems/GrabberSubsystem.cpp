// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/GrabberSubsystem.h"

GrabberSubsystem::GrabberSubsystem(
  TalonSRX& GrabberIntake,
  TalonSRX& GrabberAngle,
  frc::XboxController& stick
) :
  m_runGrabberIntakeIn(false),
  m_runGrabberIntakeOut(false),
  m_GrabberIntake{GrabberIntake},
  m_GrabberAngle{GrabberAngle},
  m_xbox{stick},
  m_EncoderCountsPerRev(2048),
  m_Position1(600),
  m_Position2(1200),
  m_Position3(1800),
  m_Counter(0)
{
}


void GrabberSubsystem::ModeInit()
{
    // Call GetRawButtonPressed to discard any button presses
    // made while the robot was disabled.
    m_xbox.GetAButton();
    StopMotor();
}

// This is mostly copied from SPARK-MAX-Examples/C++/Get and Set Parameters
void GrabberSubsystem::RobotInit()
{

}

//bool GrabberSubsystem::RunAutonomous(bool enabled)
/*
{

  if (enabled)
  {
    // m_launchDrive.Set(-m_stick.GetThrottle());
    // Super-awesome calibration.
    m_GrabberIntake.Set(0.687);
    // normal flywheel speed = 0.687
  } else {
    m_GrabberIntake.Set(0);
  }

    // periodically read voltage, temperature, and applied output and publish to SmartDashboard
    return enabled;
}
*/
bool GrabberSubsystem::RunPeriodic()
{
    // Toggle Launch state on button press.
    if (m_xbox.GetAButtonPressed())
    {
      m_runGrabberIntakeIn = !m_runGrabberIntakeIn;
    }
    if (m_xbox.GetBButtonPressed())
    {
      m_runGrabberIntakeOut = !m_runGrabberIntakeOut;
    }

    // 
    if (m_runGrabberIntakeIn)
    {
    // Throttle is connected the slider on the controller.
    // The throttle axis reads -1.0 when pressed forward.
    // Launch motor is inverted from launch motor.
      m_runGrabberIntakeOut = false;
      m_GrabberIntake.Set(ControlMode::PercentOutput, 0.5);

    } else if (m_runGrabberIntakeOut)
    {
      m_runGrabberIntakeIn = false;
      m_GrabberIntake.Set(ControlMode::PercentOutput, -0.5);
    } else {
      m_GrabberIntake.Set(ControlMode::PercentOutput, 0);
    }

    if (m_xbox.GetXButtonPressed())
    {
      m_Counter = m_Counter + 1;
    }
    if (m_Counter % 3 == 1)
    {
      m_GrabberAngle.Set(ControlMode::Position, m_Position1);
    } else if (m_Counter % 3 == 2)
    {
      m_GrabberAngle.Set(ControlMode::Position, m_Position2);
    } else 
    {
      m_GrabberAngle.Set(ControlMode::Position, m_Position3);
    }
    // periodically read voltage, temperature, and applied output and publish to SmartDashboard
    //return m_runGrabberIntake;
}


void GrabberSubsystem::StopMotor()
{
    m_runGrabberIntakeIn = false;
    m_runGrabberIntakeOut = false;
    m_GrabberIntake.Set(ControlMode::PercentOutput, 0);
}