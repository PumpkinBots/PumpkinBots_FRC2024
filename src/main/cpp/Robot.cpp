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
  armConf.MotorOutput.Inverted = true; // verified
  wristConf.MotorOutput.Inverted = false; // verified

  // limit dutycycles during calibration
  armConf.MotorOutput.PeakForwardDutyCycle = power::armPeak;  // Peak output of 10%
  armConf.MotorOutput.PeakReverseDutyCycle = -power::armPeak; // Peak output of 10%
  wristConf.MotorOutput.PeakForwardDutyCycle = power::wristPeak;  // Peak output of 10%
  wristConf.MotorOutput.PeakReverseDutyCycle = -power::wristPeak; // Peak output of 10%


  /**
   * slot0 defines the PID characteristics of MotionMagic
  */
  auto& armSlot0Conf = armConf.Slot0;
  armSlot0Conf.kS = 0.25; // Add 0.25 V output to overcome static friction
  armSlot0Conf.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
  armSlot0Conf.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
  armSlot0Conf.kP = 1.0;
  armSlot0Conf.kI = 0;
  armSlot0Conf.kD = 0;
  mmArm.Slot = 0;

  phx::configs::MotionMagicConfigs &mmArmConf = armConf.MotionMagic;
  mmArmConf.MotionMagicCruiseVelocity = 0; // max cruise velocity
  mmArmConf.MotionMagicExpo_kA = 0.01;
  mmArmConf.MotionMagicExpo_kV = 0.12;

  arm.GetConfigurator().Apply(armConf);
  armFollower.GetConfigurator().Apply(armConf);
  armFollower.SetControl(phx::controls::Follower{arm.GetDeviceID(), true}); // inverted rotation

  auto& wristSlot0Conf = wristConf.Slot0;
  wristSlot0Conf.kS = 0.25; // Add 0.25 V output to overcome static friction
  wristSlot0Conf.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
  wristSlot0Conf.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
  wristSlot0Conf.kP = 1.0; 
  wristSlot0Conf.kI = 0;
  wristSlot0Conf.kD = 0;
  mmWrist.Slot = 0;

  phx::configs::MotionMagicConfigs &mmWristConf = wristConf.MotionMagic;
  mmWristConf.MotionMagicCruiseVelocity = 0; // max cruise velocity
  mmWristConf.MotionMagicExpo_kA = 0.01;
  mmWristConf.MotionMagicExpo_kV = 0.12;

  wrist.GetConfigurator().Apply(wristConf);

  /* assume start in home position */
  arm.SetPosition(arm::home);
  wrist.SetPosition(wrist::home);

  /* Intake configuration */
  phx::configs::TalonFXConfiguration intakeConf{};
  
  /* Set rotation direction for the intake */
  /**
   * FIXME: these are RANDOMLY chosen - review literature and cad to verify
  */
  intakeConf.MotorOutput.Inverted = false; // primary intake at left when facing the intake mechanism

  // limit duty cycles during testing
  //intakeConf.MotorOutput.PeakForwardDutyCycle = 0.1;  // Peak output of 10%
  //intakeConf.MotorOutput.PeakReverseDutyCycle = -0.1; // Peak output of 10%

  intake.GetConfigurator().Apply(intakeConf);
  intakeFollower.GetConfigurator().Apply(intakeConf);
  
  /* Set up followers to follow leaders and retain the leaders' inversion settings */
  intakeFollower.SetControl(phx::controls::Follower{intake.GetDeviceID(), false});

  intakeOut.Output = power::intakePlace;
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
  slowDrive = (driveController.GetRawButtonPressed(3)) ? !slowDrive : slowDrive;
  maxSpeed = slowDrive ? 0.3 : 1.0;

  /**
   * DRIVE DIRECTION
   * Button two on the joystick toggles the reverse drive mode
  */
  driveDirection = (driveController.GetRawButtonPressed(2)) ? -driveDirection : driveDirection; // thumb button

  /**
   * SPEED
   * jitter correction: throw out any inputs less than the deadband value
  */
  const double deadband = 0.05;
  double speed = (fabs(driveController.GetY()) > deadband) ? driveDirection * driveController.GetY() : 0.0;
  
  /**
   * TURNING
   * taking half of the signed square of the twist to reduce the impact on speed (raw output should always be in the range of -1:1)
   * eg twist = -0.5 -> turn = -0.125
   *    twist = 1.0 -> turn = 0.5
   * speedTurn should slow the turning rate at speed for better controllability - can revert to turn based on driver feedback (or convert to a switched mode)
   * eg speed = 0 -> speedTurn = turn
   *    speed = 1 -> speedTurn = 0.5 * turn
  */
  double turn = (fabs(driveController.GetTwist()) > deadband) ? 0.5 * driveDirection * driveController.GetTwist() * fabs(driveController.GetTwist()) : 0.0;
  double speedTurn = turn * (1 + fabs(speed)/2);

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

  if (mechController.GetAButton()) {
    mechMode = Mech::Home;
  } else if (mechController.GetBButton()) {
    mechMode = Mech::Intake;
  } else if (mechController.GetXButton()) {
    mechMode = Mech::Release;
  } else if (mechController.GetYButton()) {
    mechMode = Mech::Climb;
  } else if (mechController.GetLeftBumper()) {
    mechMode = Mech::Delivery;
  } else if (mechController.GetRightBumper()) {
    mechMode = Mech::AmpScore;
  } else if (mechController.GetLeftStickButton()) {
    mechMode = Mech::ActivateClimbing;
  }// else if (mechController.GetStartButton()) {
   // mechMode = Mech::Manual;
   //}
  // add support for manual mode 
  // xbox.GetRightTriggerAxis() is shoot
  // -xbox.GetLeftY() is wrist moving away from home (positive angle)
  // (?)-xbox.GetRightY() is arm moving away from home (positive angle)
  // arm.SetPosition(arm::home);
  // wrist.SetPosition(wrist::home);

  // print out angular position of both arm and wrist
  //DEBUG_MSG("Arm position: " << 360 * arm.GetPosition().GetValueAsDouble() / arm::gearOut.value() << "°");
  //DEBUG_MSG("Wrist position: " << 360 * wrist.GetPosition().GetValueAsDouble() / wrist::gearOut.value() << "°");

  armMoving = arm.GetVelocity().GetValueAsDouble() != 0.0 ? true : false;
  wristMoving = wrist.GetVelocity().GetValueAsDouble() != 0.0 ? true : false;
  noteDetected = noteSensor.Get();
  //const double maxArmSpeed = slowArm ? 0.1 : 1.0; // FIXME: there are currently no user inputs to change this

  if (!armMoving && !wristMoving) { // do nothing if the mechanism is still in motion
    switch (mechMode) {
      /*
      case Mech::Manual :
        armSpeed = (fabs(mechController.GetRightY()) > deadband) ? mechController.GetRightY() : 0.0;
        wristSpeed = (fabs(mechController.GetLeftY()) > deadband) ? mechController.GetLeftY() : 0.0;
        armOut.Output = maxArmSpeed * armSpeed;
        wristOut.Output = - maxArmSpeed * wristSpeed; // FIXME is the sign on this correct or should this be handled by 'inverted'
        arm.SetControl(armOut);
        wrist.SetControl(wristOut);

        //DEBUG_MSG("Manual Mode: armOutput" << armOut.Output);
        //DEBUG_MSG("Manual Mode: wristOutput " << wristOut.Output);

        if (mechController.GetBackButton()) {
          arm.SetPosition(arm::home);
          wrist.SetPosition(wrist::home);
        }
        break;
      */

      case Mech::Home :
        intake.SetControl(phx::controls::StaticBrake{});
        arm.SetControl(mmArm.WithPosition(arm::home)); // untested ->.WithFeedForward(-0.2)); // should be dynamically calculated using arm angle
        wrist.SetControl(mmWrist.WithPosition(wrist::home));
        break;

      case Mech::Intake :
        arm.SetControl(mmArm.WithPosition(arm::intake));
        wrist.SetControl(mmWrist.WithPosition(wrist::intake));
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
        arm.SetControl(mmArm.WithPosition(arm::amp)); // untested ->.WithFeedForward(-0.2).WithFeedForward(0.2)); // should be dynamically calculated using arm angle
        wrist.SetControl(mmWrist.WithPosition(wrist::amp));
        break;

      case Mech::AmpScore :
        //arm.SetControl(mmArm.WithPosition(arm::amp)); // untested ->.WithFeedForward(-0.2).WithFeedForward(0.2)); // should be dynamically calculated using arm angle
        if (!armMoving) {
          //wrist.SetControl(mmWrist.WithPosition(wrist::amp));
        }
        if (!armMoving && !wristMoving) {
          intake.SetControl(intakeOut);
          // mechMode = Mech::Home; // reset to home
        }
        break;

      case Mech::Release :
        arm.SetControl(mmArm.WithPosition(arm::intake));
        wrist.SetControl(mmWrist.WithPosition(wrist::intake));
        if (!armMoving && !wristMoving) {
          intake.SetInverted(!intake.GetInverted()); // reverse intake motors
          intake.SetControl(intakeOut);
          intake.SetInverted(!intake.GetInverted()); // revert to standard direction
          mechMode = Mech::Home; // reset to home
        }
        break;

      case Mech::Climb :
        arm.SetControl(mmArm.WithPosition(arm::climb));
        wrist.SetControl(mmWrist.WithPosition(wrist::climb));
        break;

      case Mech::ActivateClimbing :
        //armConf.MotorOutput.PeakForwardDutyCycle = power::armClimb;  // Peak output of 10%
        //armConf.MotorOutput.PeakReverseDutyCycle = -power::armClimb; // Peak output of 10%
        arm.SetControl(mmArm.WithPosition(arm::climbDown));
        wrist.SetControl(mmWrist.WithPosition(wrist::amp));
        //armConf.MotorOutput.PeakForwardDutyCycle = power::armPeak;  // Peak output of 10%
        //armConf.MotorOutput.PeakReverseDutyCycle = -power::armPeak;
        break;
    }
  }
}

void Robot::AutonomousInit() {
    //Set timer to zero and start counting
    m_timer.Reset();
    m_timer.Start();
}

void Robot::AutonomousPeriodic() {
  double speed = 0.0;

  if (m_timer.Get() >= 0_s && m_timer.Get() <= 2_s) {
    // drive forward
    speed = 0.1;
    DEBUG_MSG("AutonomousPeriodic: speed = " << speed);
  } 
  
  leftOut.Output = speed;
  rightOut.Output = speed; 

  leftDrive.SetControl(leftOut);
  rightDrive.SetControl(rightOut);

}

void Robot::RobotPeriodic() {
  if (m_printCount++ > 10) {
    m_printCount = 0;
    DEBUG_MSG("Arm Pos: " << arm.GetPosition() << "Wrist Pos: " << wrist.GetPosition());
    DEBUG_MSG("Arm Vel: " << arm.GetVelocity() << "Wrist Vel: " << wrist.GetVelocity());
  }
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif