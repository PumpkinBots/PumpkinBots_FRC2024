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

  /**
   * DRIVE MOTOR CONFIGURATION
  */
  
  /* Drive configuration */
  phx::configs::TalonFXConfiguration leftConf{};
  phx::configs::TalonFXConfiguration rightConf{};

  /* Controls which end of the robot is front - these should always be set in opposition */
  leftConf.MotorOutput.Inverted = true;
  rightConf.MotorOutput.Inverted = false;

  /* Apply configuration */
  leftDrive.GetConfigurator().Apply(leftConf);
  leftFollower.GetConfigurator().Apply(leftConf);
  rightDrive.GetConfigurator().Apply(rightConf);
  rightFollower.GetConfigurator().Apply(rightConf);
  
  /* Set up followers to follow leaders and retain the leaders' inversion settings */
  leftFollower.SetControl(phx::controls::Follower{leftDrive.GetDeviceID(), false});
  rightFollower.SetControl(phx::controls::Follower{rightDrive.GetDeviceID(), false});

  /**
   * MECHANISM MOTOR CONFIGURATION
  */

  /* Mechanism configuration */
  phx::configs::TalonFXConfiguration armConf{};
  phx::configs::TalonFXConfiguration wristConf{};
  
  /* Set rotation direction for the arm and wrist */
  /**
   * FIXME: these are RANDOMLY chosen - review literature and cad to verify
  */
  armConf.MotorOutput.Inverted = true;
  wristConf.MotorOutput.Inverted = false;

  /**
   * slot0 defines the PID characteristics of MotionMagic
   * FIXME: characterize this properly - this is copy-pasta crap
  */
  auto& armSlot0Conf = armConf.Slot0;
  armSlot0Conf.kS = 0.05; // Add 0.25 V output to overcome static friction
  armSlot0Conf.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
  armSlot0Conf.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
  armSlot0Conf.kP = 0; // A position error of 2.5 rotations results in 12 V output
  armSlot0Conf.kI = 0; // no output for integrated error
  armSlot0Conf.kD = 0; // A velocity error of 1 rps results in 0.1 V output

  auto& mmArmConf = armConf.MotionMagic;
  mmArmConf.MotionMagicCruiseVelocity = 80;
  mmArmConf.MotionMagicAcceleration = 160;
  mmArmConf.MotionMagicJerk = 1600;
  arm.GetConfigurator().Apply(armConf);
  armFollower.GetConfigurator().Apply(armConf);

  armFollower.SetControl(phx::controls::Follower{arm.GetDeviceID(), true}); // inverted rotation

  auto& wristSlot0Conf = wristConf.Slot0;
  wristSlot0Conf.kS = 0.05; // Add 0.25 V output to overcome static friction
  wristSlot0Conf.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
  wristSlot0Conf.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
  wristSlot0Conf.kP = 0; // A position error of 2.5 rotations results in 12 V output
  wristSlot0Conf.kI = 0; // no output for integrated error
  wristSlot0Conf.kD = 0; // A velocity error of 1 rps results in 0.1 V output

  auto& mmWristConf = wristConf.MotionMagic;
  mmWristConf.MotionMagicCruiseVelocity = 80;
  mmWristConf.MotionMagicAcceleration = 160;
  mmWristConf.MotionMagicJerk = 1600;
  wrist.GetConfigurator().Apply(wristConf);

  /* assume start in home position */
  arm.SetPosition(arm::home);
  wrist.SetPosition(wrist::home);

  /* Intake configuration */
  phx::configs::TalonFXConfiguration intakeConf{};
  
  /* Set rotation direction for the arm and wrist */
  /**
   * FIXME: these are RANDOMLY chosen - review literature and cad to verify
  */
  intakeConf.MotorOutput.Inverted = false; // primary intake at left when facing the intake mechanism

  intake.GetConfigurator().Apply(intakeConf);
  intakeFollower.GetConfigurator().Apply(intakeConf);
  
  /* Set up followers to follow leaders and retain the leaders' inversion settings */
  intakeFollower.SetControl(phx::controls::Follower{intake.GetDeviceID(), false});

  intakeOut.Output = intake::intakeOut;
}

void Robot::DisabledPeriodic() {
  leftDrive.SetControl(phx::controls::NeutralOut{});
  rightDrive.SetControl(phx::controls::NeutralOut{});
  arm.SetControl(phx::controls::StaticBrake{});
  wrist.SetControl(phx::controls::StaticBrake{});
  intake.SetControl(phx::controls::NeutralOut{});
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

  leftDrive.SetControl(leftOut);
  rightDrive.SetControl(rightOut);

  /**
   * Robot starts in "home" position - arm down, and intake folded up, rollers locked
   * A (force home or finish climb): returns all systems to home. If the hooks are on the chain, this is the final climb sequence.
   * B (intake sequence): moves the wrist so the intake is ready to pick up a note, and spins rollers inward until a note is detected. Then stops the rollers, and returns to home.
   * X (release note): moves the wrist to deploy the intake, spins the rollers in reverse to "set down the note", and returns to home.
   * Y (climbing position): moves the arm up, but keeps the intake in home position to expose climbing hooks.
   * L1 (scoring part 1): moves the arm up, and adjusts the wrist so it aligns with the amp.
   * R1 (scoring part 2): spins intake motors to eject the note into the amp and returns to home
  */

  /**
   * ARM/WRIST OUTPUT
  */
  
  /* xbox input (mech) */
  // refactor? https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html
  // not a trivial refactor as states chain to each other (usually back to Mech::HOME)

  if (xbox.GetAButton()) {
    mechMode = Mech::Home;
  } else if (xbox.GetBButton()) {
    mechMode = Mech::Intake;
  } else if (xbox.GetXButton()) {
    mechMode = Mech::Release;
  } else if (xbox.GetYButton()) {
    mechMode = Mech::Climb;
  } else if (xbox.GetLeftBumper()) {
    mechMode = Mech::Delivery;
  } else if (xbox.GetRightBumper()) {
    mechMode = Mech::AmpScore;
  } else if (xbox.GetStartButton()) { // reset current position to 'home' <- this might be a bad idea
    arm.SetPosition(arm::home);
    wrist.SetPosition(wrist::home);
  }

  armMoving = arm.GetVelocity().GetValueAsDouble() != 0.0 ? true : false; // or GetRotorVelocity() ?
  wristMoving = wrist.GetVelocity().GetValueAsDouble() != 0.0 ? true : false;
  noteDetected = noteSensor.Get();

  if (!armMoving && !wristMoving) { // do nothing if the mechanism is still in motion
    switch (mechMode) {
      case Mech::Home :
        intake.SetControl(phx::controls::StaticBrake{});
        arm.SetControl(mmArm.WithPosition(arm::home).WithSlot(0));
        wrist.SetControl(mmWrist.WithPosition(wrist::home).WithSlot(0));
        break;

      case Mech::Intake :
        arm.SetControl(mmArm.WithPosition(arm::intake).WithSlot(0));
        wrist.SetControl(mmWrist.WithPosition(wrist::intake).WithSlot(0));
        if (!noteDetected && !armMoving && !wristMoving) {
          intake.SetControl(intakeOut);
        }
        if (noteDetected) {
          if (!armMoving && !wristMoving) {
            mechMode = Mech::Home; // reset to home
          }
        }
        break;

      case Mech::Delivery :
        arm.SetControl(mmArm.WithPosition(arm::amp).WithSlot(0));
        wrist.SetControl(mmWrist.WithPosition(wrist::amp).WithSlot(0));
        break;

      case Mech::AmpScore :
        arm.SetControl(mmArm.WithPosition(arm::amp).WithSlot(0));
        wrist.SetControl(mmWrist.WithPosition(wrist::amp).WithSlot(0));
        if (!armMoving && !wristMoving) {
          intake.SetControl(intakeOut);
          mechMode = Mech::Home; // reset to home
        }
        break;

      case Mech::Release :
        arm.SetControl(mmArm.WithPosition(arm::intake).WithSlot(0));
        wrist.SetControl(mmWrist.WithPosition(wrist::intake).WithSlot(0));
        if (!armMoving && !wristMoving) {
          intake.SetInverted(!intake.GetInverted()); // reverse intake motors
          intake.SetControl(intakeOut);
          intake.SetInverted(!intake.GetInverted()); // revert to standard direction
          mechMode = Mech::Home; // reset to home
        }
        break;

      case Mech::Climb :
        arm.SetControl(mmArm.WithPosition(arm::climb).WithSlot(0));
        wrist.SetControl(mmWrist.WithPosition(wrist::climb).WithSlot(0));
        break;
    }
  }

}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif