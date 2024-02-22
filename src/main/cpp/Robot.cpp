// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//core
#include <fmt/core.h>

//frc
#include <frc/TimedRobot.h>

//local
#include <Robot.h>

namespace phx = ctre::phoenix6;

/**
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
   * FIXME: voltageControl is UNTESTED and should not be necessary using *DutyCycle calls
  */
  
  /* Voltage control */
  /*
  leftLeader.SetControl(voltageControl.WithOutput(12_V));
  leftFollower.SetControl(voltageControl.WithOutput(12_V));
  rightLeader.SetControl(voltageControl.WithOutput(12_V));
  rightFollower.SetControl(voltageControl.WithOutput(12_V));
  */

  /* Drive configuration */
  phx::configs::TalonFXConfiguration leftConfiguration{};
  phx::configs::TalonFXConfiguration rightConfiguration{};

  /* Controls which end of the robot is front - these should always be set in opposition */
  leftConfiguration.MotorOutput.Inverted = true;
  rightConfiguration.MotorOutput.Inverted = false;

  /* Apply configuration */
  leftLeader.GetConfigurator().Apply(leftConfiguration);
  leftFollower.GetConfigurator().Apply(leftConfiguration);
  rightLeader.GetConfigurator().Apply(rightConfiguration);
  rightFollower.GetConfigurator().Apply(rightConfiguration);
  
  /* Set up followers to follow leaders and retain the leaders' inversion settings */
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
   * Button three on the joystick toggles slow drive mode which sets maxSpeed to 30% output
   * The robot's combined drive and turn speed are limited to the maxSpeed value.
  */
  if (joystick.GetRawButtonPressed(3)) {
    slowDrive = !slowDrive;
    fmt::print("limited maxSpeed: ", slowDrive);
  }
  const double maxSpeed = slowDrive ? 0.3 : 1.0;

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
  double turn = 0.5 * ((joystick.GetTwist()) * (fabs(joystick.GetTwist())));
  double speedTurn = turn * (-((fabs(speed)) / 2) + 1);
  
  /**
   * DRIVE OUTPUT (speed + speedTurn)
   * The existing calculation will result in even less aggressive turning at max speed due to effectively saturating the leading drive motor.
   * It might be better to cap speed + speedTurn at 1/-1
   * or it might be better to leave as-is as it further limits turning at speed
   * FIXME: explore alternate implementations of turning + speed to make it as smooth and predictable as possible for the driver
  */
  leftOut.Output = maxSpeed * (speed - speedTurn);
  rightOut.Output = maxSpeed * (speed + speedTurn); 

  leftLeader.SetControl(leftOut);
  rightLeader.SetControl(rightOut);

  /**
   * Robot starts in "home" position - arm down, and intake folded up, rollers locked
   * button 1 (intake sequence): moves the wrist so the intake is ready to pick up a note, and spins rollers inward until a note is detected. Then stops the rollers, and returns to home.
   * button 2 (scoring part 1): moves the arm up, and adjusts the wrist so it aligns with the amp.
   * button 3 (scoring part 2 or set down note):
   *   First checks if the arm is home.
   *     No --> action 1
   *     Yes --> action 2
   *     action 1: spins intake motors to eject the note into the amp and returns to home
   *     action 2: moves the wrist to deploy the intake, spins the rollers in reverse to "set down the note", and returns to home.
   * button 4 (climbing position): moves the arm up, but keeps the intake in home position to expose climbing hooks.
   * button 5 (force home or finish climb): returns all systems to home. If the hooks are on the chain, this is the final climb sequence.
  */

  /**
   * ARM/WRIST OUTPUT
  */
  switch (mechMode) {
    case Mech::Home : // BUTTON_5
      arm.SetPosition(arm::home);
      wrist.SetPosition(wrist::home);
      break;

    case Mech::Intake : // BUTTON_1
      arm.SetPosition(arm::intake);
      wrist.SetPosition(wrist::intake);
      // intake motors to pick up note (+ direction), stop intake at beam break; -- would this be a blocking call? is that good or bad?
      mechMode = Mech::Home; // reset to home for note transport
      break;

    case Mech::Delivery : // BUTTON_2
      arm.SetPosition(arm::amp);
      wrist.SetPosition(wrist::amp);
      break;

    case Mech::AmpScore :
      arm.SetPosition(arm::amp);
      wrist.SetPosition(wrist::amp);
      // intake motors to deliver note into amp (+ direction)
      mechMode = Mech::Home; // reset to home
      break;

    case Mech::Release :
      arm.SetPosition(arm::intake);
      wrist.SetPosition(wrist::intake);
      // intake motors release note onto ground (- direction)
      break;

    case Mech::Climb : // BUTTON_4
      arm.SetPosition(arm::climb);
      wrist.SetPosition(wrist::climb);
      // if BUTTON_5 {mechMode = Mech::Home}
      break;

    default : // probably unnecessary
      arm.SetPosition(arm::home);
      wrist.SetPosition(wrist::home);

  }
  //arm.SetPosition(arm::home);
  //auto& talonFXPositionSignal = arm.GetPosition(); // TODO: print this, move the motor, reprint, power off, repeat
  //fmt::print(talonFXPositionSignal.GetValueAsDouble);
  


}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif