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
   * FIXME: convert the 12_V assignment and motor inversion configuration below to a for-each loop
   *        eg with a struct of driveMotors, split between left and right
   *        this should also be extended to arm and elbow motors
   * FIXME: voltageControl is UNTESTED
  */
  
  /* Voltage control */
  leftLeader.SetControl(voltageControl.WithOutput(12_V));
  leftFollower.SetControl(voltageControl.WithOutput(12_V));
  rightLeader.SetControl(voltageControl.WithOutput(12_V));
  rightFollower.SetControl(voltageControl.WithOutput(12_V));

  /* Drive configuration */
  phx::configs::TalonFXConfiguration leftConfiguration{};
  phx::configs::TalonFXConfiguration rightConfiguration{};

  /* Controls which end of the robot is front - these should always be set in opposition */
  leftConfiguration.MotorOutput.Inverted = true;
  rightConfiguration.MotorOutput.Inverted = false;

  leftLeader.GetConfigurator().Apply(leftConfiguration);
  leftFollower.GetConfigurator().Apply(leftConfiguration);
  rightLeader.GetConfigurator().Apply(rightConfiguration);
  rightFollower.GetConfigurator().Apply(rightConfiguration);
  
  /* Set up followers to follow leaders and retain the leaders' inversion settings */
  leftFollower.SetControl(phx::controls::Follower{leftLeader.GetDeviceID(), false});
  rightFollower.SetControl(phx::controls::Follower{rightLeader.GetDeviceID(), false});

  /**
   * FIXME: we don't have Phoenix Pro, so FOC should be disabled everywhere or just ignored
   * commented out for now, should be removed.
   */
  
  /*
  if (ctre::phoenix6::IsSimulation())
  {
    leftOut.EnableFOC = false;
    rightOut.EnableFOC = false;
  }
  */

}

void Robot::DisabledPeriodic() {
  leftLeader.SetControl(phx::controls::NeutralOut{});
  rightLeader.SetControl(phx::controls::NeutralOut{});
}

void Robot::TeleopPeriodic() {
  /**
   * SLOW DRIVE
   * Button three on the joystick toggles slow drive mode.
   * In this mode, the robot's drive and turn speed are limited to the slowFactor value.
  */
  slowDrive = (joystick.GetRawButtonPressed(3)) ? !slowDrive : slowDrive;
  const double slowFactor = 0.3;

  /**
   * SPEED
   * jitter correction: throw out any inputs less than the deadband value
  */
  const double deadband = 0.05;
  double speed = (fabs(joystick.GetY()) > deadband) ? joystick.GetY() : 0.0;
  
  /**
   * TURNING
   * taking half of the signed square of the twist to reduce the impact on speed (raw output should always be in the range of -1:1)
   * eg twist = -0.5 -> turn = -0.125
   *    twist = 1.0 -> turn = 0.5
   * speedTurn should slow the turning rate at speed for better controllability - can revert to turn based on driver feedback (or convert to a switched mode)
   * eg speed = 0 -> speedTurn = turn
   *    speed = 1 -> speedTurn = 0.5 * turn
  */
  double turn = (0.5) * ((joystick.GetTwist())*(fabs(joystick.GetTwist())));
  double speedTurn = turn * (-((fabs(speed)) / 2) + 1);
  
  /**
   * DRIVE OUTPUT (speed + speedTurn)
   * The existing calculation will result in even less aggressive turning at max speed due to effectively saturating the leading drive motor.
   * It might be better to cap speed + speedTurn at 1/-1
   * or it might be better to leave as-is as it further limits turning at speed
  */
  leftOut.Output = slowDrive ? slowFactor*(speed - speedTurn) : speed - speedTurn;
  rightOut.Output = slowDrive ? slowFactor*(speed + speedTurn) : speed + speedTurn; 

  leftLeader.SetControl(leftOut);
  rightLeader.SetControl(rightOut);

}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif