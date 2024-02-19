// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//frc
#include <frc/TimedRobot.h>

//local
#include <Robot.h>

namespace phx = ctre::phoenix6;

/*
* Runs once at code initialization.
*/
void Robot::RobotInit() {

  frc::SmartDashboard::PutNumber("Auto mode", 0);

  frc::CameraServer::StartAutomaticCapture();
  frc::CameraServer::StartAutomaticCapture();

  /**
   * DRIVE MOTOR CONFIGURATION
  */

  /* configure voltage */
  /*
  // class member variable
  controls::VoltageOut m_request{0_V};

  // main robot code, command 12 V output
  m_motor.SetControl(m_request.WithOutput(12_V));
  */

  /* Configure devices */
  phx::configs::TalonFXConfiguration leftConfiguration{};
  phx::configs::TalonFXConfiguration rightConfiguration{};

  /* User can optionally change the configs, or leave it alone to perform a factory default */
  leftConfiguration.MotorOutput.Inverted = true;
  rightConfiguration.MotorOutput.Inverted = false;

  leftLeader.GetConfigurator().Apply(leftConfiguration);
  leftFollower.GetConfigurator().Apply(leftConfiguration);
  rightLeader.GetConfigurator().Apply(rightConfiguration);
  rightFollower.GetConfigurator().Apply(rightConfiguration);
    
  /* Currently in simulation, we do not support FOC, so disable it while simulating */
  if (ctre::phoenix6::IsSimulation())
  {
    leftOut.EnableFOC = false;
    rightOut.EnableFOC = false;
  }

  /* Set up followers to follow leaders */
  leftFollower.SetControl(phx::controls::Follower{leftLeader.GetDeviceID(), false});
  rightFollower.SetControl(phx::controls::Follower{rightLeader.GetDeviceID(), false});
}

void Robot::DisabledPeriodic() {
  leftLeader.SetControl(phx::controls::NeutralOut{});
  rightLeader.SetControl(phx::controls::NeutralOut{});
}

void Robot::TeleopPeriodic() {
  /**
   * SLOW DRIVE
   * Button three on the joystick toggles "slow drive" mode.
   * In this mode, the robot's drive and turn speed are limited to the slowFactor value.
  */
  if (joystick.GetRawButtonPressed(3)){
    slowDrive = !slowDrive;
  }
  const double slowFactor = 0.3;

  /**
   * SPEED
   * jitter correction: throw out any inputs less than the deadband value
  */
  const double deadband = 0.05;
  double speed = (fabs(joystick.GetY()) > deadband) ? joystick.GetY() : 0.0;
  
  /**
   * TURN
   * taking half of the signed square of the twist to reduce the impact on speed (raw output should always be in the range of -1:1)
   * eg twist = -0.5 gives a turn of -0.125
   * eg twist = 1.0 gives a turn of 0.5
  */
  double turn = (0.5) * ((joystick.GetTwist())*(fabs(joystick.GetTwist())));
  
  /*`limitedTurn` should slow the turning rate at speed for better controllability - can revert to `turn` based on driver feedback (or convert to a switched mode) */
  double limitedTurn = turn * (-((fabs(speed)) / 2) + 1);
  
  /**
   * DRIVE OUTPUT (speed + limitedTurn)
  */
  leftOut.Output = slowDrive ? slowFactor*(speed - limitedTurn) : speed - limitedTurn;
  rightOut.Output = slowDrive ? slowFactor*(speed + limitedTurn) : speed + limitedTurn; 

  leftLeader.SetControl(leftOut);
  rightLeader.SetControl(rightOut);

}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif