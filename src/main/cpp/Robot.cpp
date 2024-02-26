// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//core
#include <fmt/core.h>

//frc
#include <frc/TimedRobot.h>

//local
#include <Robot.h>
#include <iostream>

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
  phx::configs::TalonFXConfiguration leftConf{};
  phx::configs::TalonFXConfiguration rightConf{};

  /* Controls which end of the robot is front - these should always be set in opposition */
  leftConf.MotorOutput.Inverted = true;
  rightConf.MotorOutput.Inverted = false;

  /* Apply configuration */
  leftLeader.GetConfigurator().Apply(leftConf);
  leftFollower.GetConfigurator().Apply(leftConf);
  rightLeader.GetConfigurator().Apply(rightConf);
  rightFollower.GetConfigurator().Apply(rightConf);
  
  /* Set up followers to follow leaders and retain the leaders' inversion settings */
  leftFollower.SetControl(phx::controls::Follower{leftLeader.GetDeviceID(), false});
  rightFollower.SetControl(phx::controls::Follower{rightLeader.GetDeviceID(), false});

  /* set up the arm and wrist */
  phx::configs::TalonFXConfiguration armConf{};

  /* why is "slot0" necessary - extremely poor documentation */
  auto& slot0Configs = armConf.Slot0;
  slot0Configs.kS = 0.05; // Add 0.25 V output to overcome static friction
  slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
  slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
  slot0Configs.kP = 0; // A position error of 2.5 rotations results in 12 V output
  slot0Configs.kI = 0; // no output for integrated error
  slot0Configs.kD = 0; // A velocity error of 1 rps results in 0.1 V output

  auto& mmArmConf = armConf.MotionMagic;
  mmArmConf.MotionMagicCruiseVelocity = 5; //80? 100
  mmArmConf.MotionMagicAcceleration = 10; //160?
  mmArmConf.MotionMagicJerk = 1600;
  //arm.GetConfigurator().Apply(armConf);
  ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for(int i = 0; i < 5; ++i) {
    status = arm.GetConfigurator().Apply(armConf);
    if (status.IsOK()) break;
  }
  if (!status.IsOK()) {
    std::cout << "Could not configure device. Error: " << status.GetName() << std::endl;
  }
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
  
  /* xbox input (mech) */
  if (xbox.GetAButton()) { //GetRawButtonPressed(1)) { // A
    mechMode = Mech::Home;
  } else if (xbox.GetBButton()) { //.GetRawButtonPressed(2)) { //B
    mechMode = Mech::Intake;
  } else if (xbox.GetXButton()) { //.GetRawButtonPressed(3)) { //X
    mechMode = Mech::Release;
  } else if (xbox.GetYButton()) { //.GetRawButtonPressed(4)) { //Y
    mechMode = Mech::Climb;
  } else if (xbox.GetLeftBumper()) { //.GetRawButtonPressed(5)) { //L1
    mechMode = Mech::Delivery;
  } else if (xbox.GetRightBumper()) { //.GetRawButtonPressed(6)) { //R1
    mechMode = Mech::AmpScore;
  } else if (xbox.GetStartButton()) { //reset current position to 'home' <- this might be a bad idea
    arm.SetPosition(arm::home);
    wrist.SetPosition(wrist::home);
  }

  beamBreak = false;


  if (!arm.GetMotionMagicIsRunning().GetStatus()) {
    switch (mechMode) {
      case Mech::Home :
        arm.SetControl(mmArm.WithPosition(arm::home).WithSlot(0));
        //arm.SetPosition(arm::home);
        //wrist.SetControl(mmWrist.WithPosition(wrist::home));
        fmt::print("mechMode = Mech::Home - position: ", arm::home, "/n");
        break;

      case Mech::Intake :
        arm.SetControl(mmArm.WithPosition(arm::intake).WithSlot(0));
        //arm.SetPosition(arm::intake);
        //wrist.SetControl(mmWrist.WithPosition(wrist::intake));
        fmt::print("mechMode = Mech::Intake - position: ", arm::intake, "/n");
        // intake motors on
        // if (beamBreak) {
          // intake motors off
          // mechMode = Mech::Home; // reset to home for note transport
          //}
        break;

      case Mech::Delivery :
        arm.SetControl(mmArm.WithPosition(arm::amp).WithSlot(0));
        //arm.SetPosition(arm::amp);
        //wrist.SetControl(mmWrist.WithPosition(wrist::amp));
        fmt::print("mechMode = Mech::Delivery - position: ", arm::amp, "/n");
        break;

      case Mech::AmpScore :
        arm.SetControl(mmArm.WithPosition(arm::amp).WithSlot(0));
        //arm.SetPosition(arm::amp); // this should be unnecessary
        //wrist.SetControl(mmWrist.WithPosition(wrist::amp));
        fmt::print("mechMode = Mech::AmpScore - position: ", arm::amp, "/n");
        // intake motors to deliver note into amp (+ direction)
        mechMode = Mech::Home; // reset to home
        break;

      case Mech::Release :
        arm.SetControl(mmArm.WithPosition(arm::intake).WithSlot(0));
        //arm.SetPosition(arm::release);
        //wrist.SetControl(mmWrist.WithPosition(wrist::intake));
        fmt::print("mechMode = Mech::Release - position: ", arm::intake, "/n");
        // intake motors release note onto ground (- direction)
        break;

      case Mech::Climb :
        arm.SetControl(mmArm.WithPosition(arm::climb).WithSlot(0));
        //arm.SetPosition(arm::climb);
        //wrist.SetControl(mmWrist.WithPosition(wrist::climb));
        fmt::print("mechMode = Mech::Climb - position: ", arm::climb, "/n");
        break;

      default : // probably unnecessary
        arm.SetControl(mmArm.WithPosition(arm::home).WithSlot(0));
        //arm.SetPosition(arm::home);
        //wrist.SetControl(mmWrist.WithPosition(wrist::home));
        fmt::print("default \n");

    }
  }

}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif