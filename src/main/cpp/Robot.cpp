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
  /* Configure devices */
  phx::configs::TalonFXConfiguration leftConfiguration{};
  phx::configs::TalonFXConfiguration rightConfiguration{};

  /* User can optionally change the configs, or leave it alone to perform a factory default */
  leftConfiguration.MotorOutput.Inverted = false;
  rightConfiguration.MotorOutput.Inverted = true;

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
   * taking half of the signed square of the twist... must be a really slow turn by default
  */
  double turn = (0.5) * ((joystick.GetTwist())*(fabs(joystick.GetTwist())));
  double turnLimit = turn * (-((fabs(speed)) / 2) + 1);
  
  /**
   * DRIVE OUTPUT (speed + turn)
   * FIXME: not sure how to apply turnLimit here
  */
  leftOut.Output = slowDrive ? slowFactor*(speed + turn) : speed + turn; // 
  rightOut.Output = slowDrive ? slowFactor*(speed + turn) : speed + turn; // not sure how to apply turnLimit here

  leftLeader.SetControl(leftOut);
  rightLeader.SetControl(rightOut);

}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif