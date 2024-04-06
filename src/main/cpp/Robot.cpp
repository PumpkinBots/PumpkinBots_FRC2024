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

  m_chooser.SetDefaultOption(kAutoNoMove, kAutoNoMove);
  m_chooser.AddOption(kAutoLeave, kAutoLeave);
  m_chooser.AddOption(kAutoOneNoteRed, kAutoOneNoteRed);
  m_chooser.AddOption(kAutoTwoNoteRed, kAutoTwoNoteRed);
  m_chooser.AddOption(kAutoThreeNoteRed, kAutoThreeNoteRed);
  m_chooser.AddOption(kAutoFourNoteRed, kAutoFourNoteRed);
  m_chooser.AddOption(kAutoOneNoteBlue, kAutoOneNoteBlue);
  m_chooser.AddOption(kAutoTwoNoteBlue, kAutoTwoNoteBlue);
  m_chooser.AddOption(kAutoThreeNoteBlue, kAutoThreeNoteBlue);
  m_chooser.AddOption(kAutoFourNoteBlue, kAutoFourNoteBlue);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

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

  armClimbConf = armConf;
  armClimbConf.MotionMagic.MotionMagicCruiseVelocity = 15;// was 5

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
  intakeRedo.Output = power::intakeReverse;
  intakeShuttleShoot.Output = power::intakeShuttle;
}

void Robot::DisabledPeriodic() {
  leftDrive.SetControl(phx::controls::NeutralOut{});
  rightDrive.SetControl(phx::controls::NeutralOut{});
  //arm.SetControl(phx::controls::DutyCycleOut{-0.5});
  arm.SetControl(phx::controls::StaticBrake{});
  wrist.SetControl(phx::controls::StaticBrake{});
  /*if (disabledTimer == 0_s) {
    disabledTimer = m_timer.Get();
    while (disabledTimer < m_timer.Get() + 6_s) {
    //arm.SetControl(phx::controls::StaticBrake{});
    //wrist.SetControl(phx::controls::StaticBrake{});
      Mechanism();
    }
  }
  */
  intake.SetControl(phx::controls::NeutralOut{});
}

void Robot::TeleopPeriodic() {
  disabledTimer = 0_s;
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
  //driveDirection = (driveController.GetRawButtonPressed(2)) ? -driveDirection : driveDirection; // thumb button

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
  double turn = (fabs(driveController.GetTwist()) > deadband) ? 0.3 * driveDirection * driveController.GetTwist() * fabs(driveController.GetTwist()) : 0.0;
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

  frc::SmartDashboard::PutNumber("Left Speed", leftOut.Output);
  frc::SmartDashboard::PutNumber("Right Speed", rightOut.Output);
 // frc::SmartDashboard::PutNumber("Arm Position", arm.GetPosition());

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
  } else if (mechController.GetRightStickButton()) {
    mechMode = Mech::Shuttle;
  }
  
  // else if (mechController.GetStartButton()) {
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

  Mechanism();
}


void Robot::Mechanism() {
  armMoving = arm.GetVelocity().GetValueAsDouble() != 0.0 ? true : false;
  wristMoving = wrist.GetVelocity().GetValueAsDouble() != 0.0 ? true : false;
  noteDetected = noteSensor.Get();
  //const double maxArmSpeed = slowArm ? 0.1 : 1.0; // FIXME: there are currently no user inputs to change this

  if (!armMoving && !wristMoving) { // do nothing if the mechanism is still in motion
    switch (mechMode) {

      case Mech::Home :
        arm.GetConfigurator().Apply(armConf);
        intake.SetControl(phx::controls::StaticBrake{});
        arm.SetControl(mmArm.WithPosition(arm::home)); // untested ->.WithFeedForward(-0.2)); // should be dynamically calculated using arm angle
        wrist.SetControl(mmWrist.WithPosition(wrist::home));
        break;

      case Mech::Intake :
        arm.GetConfigurator().Apply(armConf);
        arm.SetControl(mmArm.WithPosition(arm::intake));
        wrist.SetControl(mmWrist.WithPosition(wrist::intake));
//        if (!noteDetected && !armMoving && !wristMoving) {
        intake.SetControl(intakeOut);
//        }
        if (noteDetected) {
          if (!armMoving && !wristMoving) {
            mechMode = Mech::Home; // reset to home
          }
        }
        break;

      case Mech::Delivery :
        arm.GetConfigurator().Apply(armConf);
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
        arm.GetConfigurator().Apply(armConf);
        arm.SetControl(mmArm.WithPosition(arm::intake));
        wrist.SetControl(mmWrist.WithPosition(wrist::intake));
        intake.SetControl(intakeRedo);

        /*if (!armMoving && !wristMoving) {
          if (outputTimer == 0_s) {
            outputTimer = m_timer.Get();
            intake.SetInverted(!intake.GetInverted()); // reverse intake motors
            intake.SetControl(intakeOut);
          } else if (outputTimer + 1_s <= m_timer.Get()) { // modify delay for sufficient "eject" time as necessary
            outputTimer = 0_s;
            intake.SetInverted(!intake.GetInverted()); // revert to standard direction
            mechMode = Mech::Home; // reset to home
          }
        }
        */
        break;

      case Mech::Climb :
        arm.GetConfigurator().Apply(armConf);
        arm.SetControl(mmArm.WithPosition(arm::climb));
        wrist.SetControl(mmWrist.WithPosition(wrist::climb));
        break;

      case Mech::ActivateClimbing :
        arm.GetConfigurator().Apply(armClimbConf);
        arm.SetControl(mmArm.WithPosition(arm::climbDown));
        wrist.SetControl(mmWrist.WithPosition(wrist::amp));
        break;

      case Mech::Shuttle :
        arm.GetConfigurator().Apply(armConf);
        arm.SetControl(mmArm.WithPosition(arm::home));
        wrist.SetControl(mmWrist.WithPosition(wrist::shuttle));
        if (!wristMoving and !armMoving) {
          intake.SetControl(intakeShuttleShoot);
        }
    }
  }
  
}

void Robot::AutonomousInit() {
  //Set timer to zero and start counting
  m_timer.Reset();
  m_timer.Start();
  double rightSpeed = 0.0;
  double leftSpeed = 0.0;
  m_autoSelected = m_chooser.GetSelected();
  fmt::print("Auto selected: {}\n", m_autoSelected);

  
  if (m_autoSelected == kAutoLeave) {  // exit / go forward code
    if (m_timer.Get() >= 0_s && m_timer.Get() <= 2_s) {
      // drive forward
      rightSpeed = -0.1;
      leftSpeed = -0.1;
    }
  }
  else if (m_autoSelected == kAutoOneNoteRed) { // one note red
    if (m_timer.Get() >= 0_s && m_timer.Get() <= 0.6_s) {
      // drive forward
      rightSpeed = -0.15;
      leftSpeed = -0.15;
    }
    if (m_timer.Get() >= 0.6_s && m_timer.Get() <= 1.35_s) {
      // turn right
      leftSpeed = -0.21;
      rightSpeed = 0.21;
    }
    if (m_timer.Get() >= 1.35_s && m_timer.Get() <= 2.45_s) {
      // drive towards amp
      leftSpeed = -0.12;
      rightSpeed = -0.12;
      // raise arm to scoring position
      arm.SetControl(mmArm.WithPosition(arm::amp)); // untested ->.WithFeedForward(-0.2).WithFeedForward(0.2)); // should be dynamically calculated using arm angle
      wrist.SetControl(mmWrist.WithPosition(wrist::amp));
    }
    if (m_timer.Get() >= 2.45_s && m_timer.Get() <= 2.95_s) {
      // spin intake to score
      intake.SetControl(intakeOut);
    }
    if (m_timer.Get() >= 2.95_s && m_timer.Get() <= 3.45_s) {
      // stop intake after scored
      intake.SetControl(phx::controls::StaticBrake{});
      // lower arm and wrist to intake position
      arm.SetControl(mmArm.WithPosition(arm::home));
      wrist.SetControl(mmWrist.WithPosition(wrist::home));
    }
  }
  else if (m_autoSelected == kAutoTwoNoteRed) { // two note red
    if (m_timer.Get() >= 0_s && m_timer.Get() <= 0.6_s) {
      // drive forward
      rightSpeed = -0.15;
      leftSpeed = -0.15;
    }
    if (m_timer.Get() >= 0.6_s && m_timer.Get() <= 1.35_s) {
      // turn right
      leftSpeed = -0.21;
      rightSpeed = 0.21;
    }
    if (m_timer.Get() >= 1.35_s && m_timer.Get() <= 2.45_s) {
      // drive towards amp
      leftSpeed = -0.12;
      rightSpeed = -0.12;
      // raise arm to scoring position
      arm.SetControl(mmArm.WithPosition(arm::amp)); // untested ->.WithFeedForward(-0.2).WithFeedForward(0.2)); // should be dynamically calculated using arm angle
      wrist.SetControl(mmWrist.WithPosition(wrist::amp));
    }
    if (m_timer.Get() >= 2.45_s && m_timer.Get() <= 2.95_s) {
      // spin intake to score
      intake.SetControl(intakeOut);
    }
    if (m_timer.Get() >= 2.95_s && m_timer.Get() <= 3.45_s) {
      // stop intake after scored
      intake.SetControl(phx::controls::StaticBrake{});
      // lower arm and wrist to intake position
      arm.SetControl(mmArm.WithPosition(arm::intake));
      wrist.SetControl(mmWrist.WithPosition(wrist::intake));
      // drive away from amp
      leftSpeed = 0.28;
      rightSpeed = 0.28;
    }
    if (m_timer.Get() >= 3.45_s && m_timer.Get() <= 4.2_s) {
      // turn towards second note
      leftSpeed = -0.16;
      rightSpeed = 0.16;

    }
    if (m_timer.Get() >= 4.2_s && m_timer.Get() <= 5.1_s) {
      // stop roller after picked up second note
      wrist.SetControl(mmWrist.WithPosition(wrist::intake));
      leftSpeed = 0.22;
      rightSpeed = 0.22;
    }
    if (m_timer.Get() >= 4.2_s && m_timer.Get() <= 5.15_s) {
      // stop roller after picked up second note
      intake.SetControl(intakeOut);
    }
    if (m_timer.Get() >= 5.1_s && m_timer.Get() <= 6.05_s) {
      leftSpeed = -0.19;
      rightSpeed = -0.19;
    }
    if (m_timer.Get() >= 5.15_s && m_timer.Get() <= 5.65_s) {
      intake.SetControl(phx::controls::StaticBrake{});
    }
    if (m_timer.Get() >= 6.05_s && m_timer.Get() <= 6.9_s) {
      leftSpeed = 0.2;
      rightSpeed = -0.2;
    }
    if (m_timer.Get() >= 6.9_s && m_timer.Get() <= 8.35_s) {
      leftSpeed = -0.1;
      rightSpeed = -0.1;
      arm.SetControl(mmArm.WithPosition(arm::amp)); // untested ->.WithFeedForward(-0.2)); // should be dynamically calculated using arm angle
      wrist.SetControl(mmWrist.WithPosition(wrist::amp));
    }
    if (m_timer.Get() >= 8.35_s && m_timer.Get() <= 9.35_s) {
      intake.SetControl(intakeOut);
    }
    if (m_timer.Get() >= 9.35_s && m_timer.Get() <= 9.85_s) {
      arm.SetControl(mmArm.WithPosition(arm::home)); // untested ->.WithFeedForward(-0.2)); // should be dynamically calculated using arm angle
      wrist.SetControl(mmWrist.WithPosition(wrist::home));
      intake.SetControl(phx::controls::StaticBrake{});
    }


    
  }
  else if (m_autoSelected == kAutoThreeNoteRed) { // three note red
    if (m_timer.Get() >= 0_s && m_timer.Get() <= 0.6_s) {
      // drive forward
      rightSpeed = -0.1;
      leftSpeed = -0.1;
    }
    if (m_timer.Get() >= 0.6_s && m_timer.Get() <= 1.75_s) {
      // turn right
      leftSpeed = -0.1;
      rightSpeed = 0.1;
    }
    if (m_timer.Get() >= 1.75_s && m_timer.Get() <= 2.5_s) {
      // drive towards amp
      leftSpeed = -0.1;
      rightSpeed = -0.1;
      // raise arm to scoring position
      arm.SetControl(mmArm.WithPosition(arm::amp)); // untested ->.WithFeedForward(-0.2).WithFeedForward(0.2)); // should be dynamically calculated using arm angle
      wrist.SetControl(mmWrist.WithPosition(wrist::amp));
    }
    if (m_timer.Get() >= 2.6_s && m_timer.Get() <= 3.0_s) {
      // spin intake to score
      intake.SetControl(intakeOut);
    }
    if (m_timer.Get() >= 3.0_s && m_timer.Get() <= 5.5_s) {
      // stop intake after scored
      intake.SetControl(phx::controls::StaticBrake{});
      // lower arm and wrist to home position
      arm.SetControl(mmArm.WithPosition(arm::home));
      wrist.SetControl(mmWrist.WithPosition(wrist::home));
      // drive towards second note
      leftSpeed = -0.01;
      rightSpeed = 0.12;
    }
    if (m_timer.Get() >= 5.25_s && m_timer.Get() <= 6.25_s) {
      // pick up second note
      intake.SetControl(intakeOut);
    }
    if (m_timer.Get() >= 6.25_s && m_timer.Get() <= 6.5_s) {
      // stop roller after picked up second note
      intake.SetControl(phx::controls::StaticBrake{});
    }
    if (m_timer.Get() >= 6.0_s && m_timer.Get() <= 8.75_s) {
      // wrist and arm to home positon
      arm.SetControl(mmArm.WithPosition(arm::home));
      wrist.SetControl(mmWrist.WithPosition(wrist::home));
      // drive to amp
      leftSpeed = 0.01;
      rightSpeed = -0.11;
    }
    if (m_timer.Get() >= 8.75_s && m_timer.Get() <= 9.25_s) {
      // bring arm and wrist up to amp position
      arm.SetControl(mmArm.WithPosition(arm::amp));
      wrist.SetControl(mmWrist.WithPosition(wrist::amp));
    }
    if (m_timer.Get() >= 9.5_s && m_timer.Get() <= 10.0_s) {
      // shoot second note
      intake.SetControl(intakeOut);
    }
    if (m_timer.Get() >= 10.0_s && m_timer.Get() <= 10.5_s) {
      // stop intake rollers
      intake.SetControl(phx::controls::StaticBrake{});
      // bring wrist and arm to intake position
      arm.SetControl(mmArm.WithPosition(arm::intake)); // untested ->.WithFeedForward(-0.2)); // should be dynamically calculated using arm angle
      wrist.SetControl(mmWrist.WithPosition(wrist::intake));
    }
    if (m_timer.Get() >= 10.5_s && m_timer.Get() <= 12.25_s) {
      // go to third note
      leftSpeed = 0.15;
      rightSpeed = 0.275;
      // activate intake rollers
      intake.SetControl(intakeOut);
    }
    if (m_timer.Get() >= 12.25_s && m_timer.Get() <= 14.0_s) {
      // stop intake rollers
      intake.SetControl(phx::controls::StaticBrake{});
      // go back to amp
      leftSpeed = -0.15;
      rightSpeed = -0.275;
    }
    if (m_timer.Get() >= 14.0_s && m_timer.Get() <= 14.5_s) {
      // bring arm and wrist to amp position
      arm.SetControl(mmArm.WithPosition(arm::amp)); // untested ->.WithFeedForward(-0.2)); // should be dynamically calculated using arm angle
      wrist.SetControl(mmWrist.WithPosition(wrist::amp));
    }
    if (m_timer.Get() >= 14.5_s && m_timer.Get() <= 15.0_s) {
      // shoot third note
      intake.SetControl(intakeOut);
    }
  }
  else if (m_autoSelected == kAutoFourNoteRed) { // four note red

  }
  else if (m_autoSelected == kAutoOneNoteBlue) { // one note blue
    if (m_timer.Get() >= 0_s && m_timer.Get() <= 0.6_s) {
      // drive forward
      rightSpeed = -0.15;
      leftSpeed = -0.15;
    }
    if (m_timer.Get() >= 0.6_s && m_timer.Get() <= 1.35_s) {
      // turn right
      leftSpeed = 0.21;
      rightSpeed = -0.21;
    }
    if (m_timer.Get() >= 1.35_s && m_timer.Get() <= 2.45_s) {
      // drive towards amp
      leftSpeed = -0.12;
      rightSpeed = -0.12;
      // raise arm to scoring position
      arm.SetControl(mmArm.WithPosition(arm::amp)); // untested ->.WithFeedForward(-0.2).WithFeedForward(0.2)); // should be dynamically calculated using arm angle
      wrist.SetControl(mmWrist.WithPosition(wrist::amp));
    }
    if (m_timer.Get() >= 2.45_s && m_timer.Get() <= 2.95_s) {
      // spin intake to score
      intake.SetControl(intakeOut);
    }
    if (m_timer.Get() >= 2.95_s && m_timer.Get() <= 3.45_s) {
      // stop intake after scored
      intake.SetControl(phx::controls::StaticBrake{});
      // lower arm and wrist to intake position
      arm.SetControl(mmArm.WithPosition(arm::home));
      wrist.SetControl(mmWrist.WithPosition(wrist::home));
    }
  }
  else if (m_autoSelected == kAutoTwoNoteBlue) { // two note blue
    if (m_timer.Get() >= 0_s && m_timer.Get() <= 0.6_s) {
      // drive forward
      rightSpeed = -0.15;
      leftSpeed = -0.15;
    }
    if (m_timer.Get() >= 0.6_s && m_timer.Get() <= 1.35_s) {
      // turn right
      leftSpeed = 0.21;
      rightSpeed = -0.21;
    }
    if (m_timer.Get() >= 1.35_s && m_timer.Get() <= 2.45_s) {
      // drive towards amp
      leftSpeed = -0.12;
      rightSpeed = -0.12;
      // raise arm to scoring position
      arm.SetControl(mmArm.WithPosition(arm::amp)); // untested ->.WithFeedForward(-0.2).WithFeedForward(0.2)); // should be dynamically calculated using arm angle
      wrist.SetControl(mmWrist.WithPosition(wrist::amp));
    }
    if (m_timer.Get() >= 2.45_s && m_timer.Get() <= 2.95_s) {
      // spin intake to score
      intake.SetControl(intakeOut);
    }
    if (m_timer.Get() >= 2.95_s && m_timer.Get() <= 3.45_s) {
      // stop intake after scored
      intake.SetControl(phx::controls::StaticBrake{});
      // lower arm and wrist to intake position
      arm.SetControl(mmArm.WithPosition(arm::intake));
      wrist.SetControl(mmWrist.WithPosition(wrist::intake));
      // drive towards second note
      leftSpeed = 0.28;
      rightSpeed = 0.28;
    }
    if (m_timer.Get() >= 3.45_s && m_timer.Get() <= 4.2_s) {
      // turn towards second note
      leftSpeed = 0.16;
      rightSpeed = -0.16;

    }
    if (m_timer.Get() >= 4.2_s && m_timer.Get() <= 5.1_s) {
      // pick up second note
      wrist.SetControl(mmWrist.WithPosition(wrist::intake));
      leftSpeed = 0.22;
      rightSpeed = 0.22;
    }
    if (m_timer.Get() >= 4.2_s && m_timer.Get() <= 5.15_s) {
      // pick up second note
      intake.SetControl(intakeOut);
    }
    if (m_timer.Get() >= 5.1_s && m_timer.Get() <= 6.05_s) {
      // move away from second note
      leftSpeed = -0.19;
      rightSpeed = -0.19;
    }
    if (m_timer.Get() >= 5.15_s && m_timer.Get() <= 5.65_s) {
      // stop rollers after second note picked up
      intake.SetControl(phx::controls::StaticBrake{});
    }
    if (m_timer.Get() >= 6.05_s && m_timer.Get() <= 6.9_s) {
      // turn towards amp to score second note
      leftSpeed = -0.2;
      rightSpeed = 0.2;
    }
    if (m_timer.Get() >= 6.9_s && m_timer.Get() <= 8.35_s) {
      // drive and score second note
      leftSpeed = -0.1;
      rightSpeed = -0.1;
      arm.SetControl(mmArm.WithPosition(arm::amp)); // untested ->.WithFeedForward(-0.2)); // should be dynamically calculated using arm angle
      wrist.SetControl(mmWrist.WithPosition(wrist::amp));
    }
    if (m_timer.Get() >= 8.35_s && m_timer.Get() <= 9.35_s) {
      // score second note
      intake.SetControl(intakeOut);
    }
    if (m_timer.Get() >= 9.35_s && m_timer.Get() <= 9.85_s) {
      // home position to end auto
      arm.SetControl(mmArm.WithPosition(arm::home)); // untested ->.WithFeedForward(-0.2)); // should be dynamically calculated using arm angle
      wrist.SetControl(mmWrist.WithPosition(wrist::home));
      intake.SetControl(phx::controls::StaticBrake{});
    }
  }
  else if (m_autoSelected == kAutoThreeNoteBlue) { // three note blue
    // no code
  }
  else if (m_autoSelected == kAutoFourNoteBlue) { // four note blue
    // no code
  }
  else if (m_autoSelected == kAutoNoMove) { // no move
    // no code
  }
  else { // shouldn't ever run, default to no movement
    // no code
  }

  
}

void Robot::AutonomousPeriodic() {
  double rightSpeed = 0.0;
  double leftSpeed = 0.0;
  bool driveStaticBrake = false;
  // sets up armMoving, wristMoving, and noteDetected variables to be accessible in autonomous code
  armMoving = arm.GetVelocity().GetValueAsDouble() != 0.0 ? true : false;
  wristMoving = wrist.GetVelocity().GetValueAsDouble() != 0.0 ? true : false;
  noteDetected = noteSensor.Get();

/*
  if (!sideOfField) {
    if (m_timer.Get() >= 0_s && m_timer.Get() <= 0.6_s) {
      // drive forward
      rightSpeed = -0.1;
      leftSpeed = -0.1;
      DEBUG_MSG("AutonomousPeriodic: speed = " << leftSpeed << " " << rightSpeed);
    }
    if (m_timer.Get() >= 0.6_s && m_timer.Get() <= 1.75_s){
      leftSpeed = -0.1;
      rightSpeed = 0.1;
    }
    if (m_timer.Get() >= 1.75_s && m_timer.Get() <= 2.5_s){
      leftSpeed = -0.1;
      rightSpeed = -0.1;
      arm.SetControl(mmArm.WithPosition(arm::amp)); // untested ->.WithFeedForward(-0.2).WithFeedForward(0.2)); // should be dynamically calculated using arm angle
      wrist.SetControl(mmWrist.WithPosition(wrist::amp));
    }
    if (m_timer.Get() >= 2.6_s && m_timer.Get() <= 3.0_s){
      intake.SetControl(intakeOut);
    }
    if (m_timer.Get() >= 3.0_s && m_timer.Get() <= 5.5_s){
      intake.SetControl(phx::controls::StaticBrake{});
      leftSpeed = -0.01;
      rightSpeed = 0.12;
      arm.SetControl(mmArm.WithPosition(arm::intake));
      wrist.SetControl(mmWrist.WithPosition(wrist::intake));
    }
    if (m_timer.Get() >= 5.25_s && m_timer.Get() <= 6.25_s){
      intake.SetControl(intakeOut);
    }
    if (m_timer.Get() >= 6.25_s && m_timer.Get() <= 6.5_s){
      intake.SetControl(phx::controls::StaticBrake{});
    }
    if (m_timer.Get() >= 6.0_s && m_timer.Get() <= 8.75_s){
      arm.SetControl(mmArm.WithPosition(arm::home));
      wrist.SetControl(mmWrist.WithPosition(wrist::home));
      leftSpeed = 0.01;
      rightSpeed = -0.11;
    }
    if (m_timer.Get() >= 8.75_s && m_timer.Get() <= 9.25_s){
      arm.SetControl(mmArm.WithPosition(arm::amp));
      wrist.SetControl(mmWrist.WithPosition(wrist::amp));
    }
    if (m_timer.Get() >= 9.5_s && m_timer.Get() <= 10.0_s){
      intake.SetControl(intakeOut);
    }
    if (m_timer.Get() >= 10.0_s && m_timer.Get() <= 10.5_s){
      intake.SetControl(phx::controls::StaticBrake{});
      arm.SetControl(mmArm.WithPosition(arm::intake)); // untested ->.WithFeedForward(-0.2)); // should be dynamically calculated using arm angle
      wrist.SetControl(mmWrist.WithPosition(wrist::intake));
    }
    if (m_timer.Get() >= 10.5_s && m_timer.Get() <= 12.25_s){
      leftSpeed = 0.15;
      rightSpeed = 0.275;
      intake.SetControl(intakeOut);
    }
    if (m_timer.Get() >= 12.25_s && m_timer.Get() <= 14.0_s){
      intake.SetControl(phx::controls::StaticBrake{});
      leftSpeed = -0.15;
      rightSpeed = -0.275;
    }
    if (m_timer.Get() >= 14.0_s && m_timer.Get() <= 14.5_s){
      arm.SetControl(mmArm.WithPosition(arm::amp)); // untested ->.WithFeedForward(-0.2)); // should be dynamically calculated using arm angle
      wrist.SetControl(mmWrist.WithPosition(wrist::amp));
    }
    if (m_timer.Get() >= 14.5_s && m_timer.Get() <= 15.0_s){
      intake.SetControl(intakeOut);
    }
  }
*/

  if (m_autoSelected == kAutoLeave) {  // exit / go forward code
    if (m_timer.Get() >= 0_s && m_timer.Get() <= 2_s) {
      // drive forward
      rightSpeed = -0.1;
      leftSpeed = -0.1;
    }
  }
  else if (m_autoSelected == kAutoOneNoteRed) { // one note red
    if (m_timer.Get() >= 0_s && m_timer.Get() <= 0.6_s) {
      // drive forward
      rightSpeed = -0.15;
      leftSpeed = -0.15;
    }
    if (m_timer.Get() >= 0.6_s && m_timer.Get() <= 1.35_s) {
      // turn right
      leftSpeed = -0.21;
      rightSpeed = 0.21;
    }
    if (m_timer.Get() >= 1.35_s && m_timer.Get() <= 2.45_s) {
      // drive towards amp
      leftSpeed = -0.12;
      rightSpeed = -0.12;
      // raise arm to scoring position
      arm.SetControl(mmArm.WithPosition(arm::amp)); // untested ->.WithFeedForward(-0.2).WithFeedForward(0.2)); // should be dynamically calculated using arm angle
      wrist.SetControl(mmWrist.WithPosition(wrist::amp));
    }
    if (m_timer.Get() >= 2.45_s && m_timer.Get() <= 2.95_s) {
      // spin intake to score
      intake.SetControl(intakeOut);
    }
    if (m_timer.Get() >= 2.95_s && m_timer.Get() <= 3.45_s) {
      // stop intake after scored
      intake.SetControl(phx::controls::StaticBrake{});
      // lower arm and wrist to intake position
      arm.SetControl(mmArm.WithPosition(arm::home));
      wrist.SetControl(mmWrist.WithPosition(wrist::home));
    }
  }
  else if (m_autoSelected == kAutoTwoNoteRed) { // two note red
    if (m_timer.Get() >= 0_s && m_timer.Get() <= 0.6_s) {
      // drive forward
      rightSpeed = -0.15;
      leftSpeed = -0.15;
    }
    if (m_timer.Get() >= 0.6_s && m_timer.Get() <= 1.35_s) {
      // turn right
      leftSpeed = -0.21;
      rightSpeed = 0.21;
    }
    if (m_timer.Get() >= 1.35_s && m_timer.Get() <= 2.45_s) {
      // drive towards amp
      leftSpeed = -0.12;
      rightSpeed = -0.12;
      // raise arm to scoring position
      arm.SetControl(mmArm.WithPosition(arm::amp)); // untested ->.WithFeedForward(-0.2).WithFeedForward(0.2)); // should be dynamically calculated using arm angle
      wrist.SetControl(mmWrist.WithPosition(wrist::amp));
    }
    if (m_timer.Get() >= 2.45_s && m_timer.Get() <= 2.95_s) {
      // spin intake to score
      intake.SetControl(intakeOut);
    }
    if (m_timer.Get() >= 2.95_s && m_timer.Get() <= 3.45_s) {
      // stop intake after scored
      intake.SetControl(phx::controls::StaticBrake{});
      // lower arm and wrist to intake position
      arm.SetControl(mmArm.WithPosition(arm::intake));
      wrist.SetControl(mmWrist.WithPosition(wrist::intake));
      // drive away from amp
      leftSpeed = 0.28;
      rightSpeed = 0.28;
    }
    if (m_timer.Get() >= 3.45_s && m_timer.Get() <= 4.2_s) {
      // turn towards second note
      leftSpeed = -0.16;
      rightSpeed = 0.16;

    }
    if (m_timer.Get() >= 4.2_s && m_timer.Get() <= 5.1_s) {
      // stop roller after picked up second note
      wrist.SetControl(mmWrist.WithPosition(wrist::intake));
      leftSpeed = 0.22;
      rightSpeed = 0.22;
    }
    if (m_timer.Get() >= 4.2_s && m_timer.Get() <= 5.15_s) {
      // stop roller after picked up second note
      intake.SetControl(intakeOut);
    }
    if (m_timer.Get() >= 5.1_s && m_timer.Get() <= 6.05_s) {
      leftSpeed = -0.19;
      rightSpeed = -0.19;
    }
    if (m_timer.Get() >= 5.15_s && m_timer.Get() <= 5.65_s) {
      intake.SetControl(phx::controls::StaticBrake{});
    }
    if (m_timer.Get() >= 6.05_s && m_timer.Get() <= 6.9_s) {
      leftSpeed = 0.2;
      rightSpeed = -0.2;
    }
    if (m_timer.Get() >= 6.9_s && m_timer.Get() <= 8.35_s) {
      leftSpeed = -0.1;
      rightSpeed = -0.1;
      arm.SetControl(mmArm.WithPosition(arm::amp)); // untested ->.WithFeedForward(-0.2)); // should be dynamically calculated using arm angle
      wrist.SetControl(mmWrist.WithPosition(wrist::amp));
    }
    if (m_timer.Get() >= 8.35_s && m_timer.Get() <= 9.35_s) {
      intake.SetControl(intakeOut);
    }
    if (m_timer.Get() >= 9.35_s && m_timer.Get() <= 9.85_s) {
      arm.SetControl(mmArm.WithPosition(arm::home)); // untested ->.WithFeedForward(-0.2)); // should be dynamically calculated using arm angle
      wrist.SetControl(mmWrist.WithPosition(wrist::home));
      intake.SetControl(phx::controls::StaticBrake{});
    }


    
  }
  else if (m_autoSelected == kAutoThreeNoteRed) { // three note red
    if (m_timer.Get() >= 0_s && m_timer.Get() <= 0.6_s) {
      // drive forward
      rightSpeed = -0.1;
      leftSpeed = -0.1;
    }
    if (m_timer.Get() >= 0.6_s && m_timer.Get() <= 1.75_s) {
      // turn right
      leftSpeed = -0.1;
      rightSpeed = 0.1;
    }
    if (m_timer.Get() >= 1.75_s && m_timer.Get() <= 2.5_s) {
      // drive towards amp
      leftSpeed = -0.1;
      rightSpeed = -0.1;
      // raise arm to scoring position
      arm.SetControl(mmArm.WithPosition(arm::amp)); // untested ->.WithFeedForward(-0.2).WithFeedForward(0.2)); // should be dynamically calculated using arm angle
      wrist.SetControl(mmWrist.WithPosition(wrist::amp));
    }
    if (m_timer.Get() >= 2.6_s && m_timer.Get() <= 3.0_s) {
      // spin intake to score
      intake.SetControl(intakeOut);
    }
    if (m_timer.Get() >= 3.0_s && m_timer.Get() <= 5.5_s) {
      // stop intake after scored
      intake.SetControl(phx::controls::StaticBrake{});
      // lower arm and wrist to home position
      arm.SetControl(mmArm.WithPosition(arm::home));
      wrist.SetControl(mmWrist.WithPosition(wrist::home));
      // drive towards second note
      leftSpeed = -0.01;
      rightSpeed = 0.12;
    }
    if (m_timer.Get() >= 5.25_s && m_timer.Get() <= 6.25_s) {
      // pick up second note
      intake.SetControl(intakeOut);
    }
    if (m_timer.Get() >= 6.25_s && m_timer.Get() <= 6.5_s) {
      // stop roller after picked up second note
      intake.SetControl(phx::controls::StaticBrake{});
    }
    if (m_timer.Get() >= 6.0_s && m_timer.Get() <= 8.75_s) {
      // wrist and arm to home positon
      arm.SetControl(mmArm.WithPosition(arm::home));
      wrist.SetControl(mmWrist.WithPosition(wrist::home));
      // drive to amp
      leftSpeed = 0.01;
      rightSpeed = -0.11;
    }
    if (m_timer.Get() >= 8.75_s && m_timer.Get() <= 9.25_s) {
      // bring arm and wrist up to amp position
      arm.SetControl(mmArm.WithPosition(arm::amp));
      wrist.SetControl(mmWrist.WithPosition(wrist::amp));
    }
    if (m_timer.Get() >= 9.5_s && m_timer.Get() <= 10.0_s) {
      // shoot second note
      intake.SetControl(intakeOut);
    }
    if (m_timer.Get() >= 10.0_s && m_timer.Get() <= 10.5_s) {
      // stop intake rollers
      intake.SetControl(phx::controls::StaticBrake{});
      // bring wrist and arm to intake position
      arm.SetControl(mmArm.WithPosition(arm::intake)); // untested ->.WithFeedForward(-0.2)); // should be dynamically calculated using arm angle
      wrist.SetControl(mmWrist.WithPosition(wrist::intake));
    }
    if (m_timer.Get() >= 10.5_s && m_timer.Get() <= 12.25_s) {
      // go to third note
      leftSpeed = 0.15;
      rightSpeed = 0.275;
      // activate intake rollers
      intake.SetControl(intakeOut);
    }
    if (m_timer.Get() >= 12.25_s && m_timer.Get() <= 14.0_s) {
      // stop intake rollers
      intake.SetControl(phx::controls::StaticBrake{});
      // go back to amp
      leftSpeed = -0.15;
      rightSpeed = -0.275;
    }
    if (m_timer.Get() >= 14.0_s && m_timer.Get() <= 14.5_s) {
      // bring arm and wrist to amp position
      arm.SetControl(mmArm.WithPosition(arm::amp)); // untested ->.WithFeedForward(-0.2)); // should be dynamically calculated using arm angle
      wrist.SetControl(mmWrist.WithPosition(wrist::amp));
    }
    if (m_timer.Get() >= 14.5_s && m_timer.Get() <= 15.0_s) {
      // shoot third note
      intake.SetControl(intakeOut);
    }
  }
  else if (m_autoSelected == kAutoFourNoteRed) { // four note red

  }
  else if (m_autoSelected == kAutoOneNoteBlue) { // one note blue
    if (m_timer.Get() >= 0_s && m_timer.Get() <= 0.6_s) {
      // drive forward
      rightSpeed = -0.15;
      leftSpeed = -0.15;
    }
    if (m_timer.Get() >= 0.6_s && m_timer.Get() <= 1.35_s) {
      // turn right
      leftSpeed = 0.21;
      rightSpeed = -0.21;
    }
    if (m_timer.Get() >= 1.35_s && m_timer.Get() <= 2.45_s) {
      // drive towards amp
      leftSpeed = -0.12;
      rightSpeed = -0.12;
      // raise arm to scoring position
      arm.SetControl(mmArm.WithPosition(arm::amp)); // untested ->.WithFeedForward(-0.2).WithFeedForward(0.2)); // should be dynamically calculated using arm angle
      wrist.SetControl(mmWrist.WithPosition(wrist::amp));
    }
    if (m_timer.Get() >= 2.45_s && m_timer.Get() <= 2.95_s) {
      // spin intake to score
      intake.SetControl(intakeOut);
    }
    if (m_timer.Get() >= 2.95_s && m_timer.Get() <= 3.45_s) {
      // stop intake after scored
      intake.SetControl(phx::controls::StaticBrake{});
      // lower arm and wrist to intake position
      arm.SetControl(mmArm.WithPosition(arm::home));
      wrist.SetControl(mmWrist.WithPosition(wrist::home));
    }
  }
  else if (m_autoSelected == kAutoTwoNoteBlue) { // two note blue
    if (m_timer.Get() >= 0_s && m_timer.Get() <= 0.6_s) {
      // drive forward
      rightSpeed = -0.15;
      leftSpeed = -0.15;
    }
    if (m_timer.Get() >= 0.6_s && m_timer.Get() <= 1.35_s) {
      // turn right
      leftSpeed = 0.21;
      rightSpeed = -0.21;
    }
    if (m_timer.Get() >= 1.35_s && m_timer.Get() <= 2.45_s) {
      // drive towards amp
      leftSpeed = -0.12;
      rightSpeed = -0.12;
      // raise arm to scoring position
      arm.SetControl(mmArm.WithPosition(arm::amp)); // untested ->.WithFeedForward(-0.2).WithFeedForward(0.2)); // should be dynamically calculated using arm angle
      wrist.SetControl(mmWrist.WithPosition(wrist::amp));
    }
    if (m_timer.Get() >= 2.45_s && m_timer.Get() <= 2.95_s) {
      // spin intake to score
      intake.SetControl(intakeOut);
    }
    if (m_timer.Get() >= 2.95_s && m_timer.Get() <= 3.45_s) {
      // stop intake after scored
      intake.SetControl(phx::controls::StaticBrake{});
      // lower arm and wrist to intake position
      arm.SetControl(mmArm.WithPosition(arm::intake));
      wrist.SetControl(mmWrist.WithPosition(wrist::intake));
      // drive towards second note
      leftSpeed = 0.28;
      rightSpeed = 0.28;
    }
    if (m_timer.Get() >= 3.45_s && m_timer.Get() <= 4.2_s) {
      // turn towards second note
      leftSpeed = 0.16;
      rightSpeed = -0.16;

    }
    if (m_timer.Get() >= 4.2_s && m_timer.Get() <= 5.1_s) {
      // pick up second note
      wrist.SetControl(mmWrist.WithPosition(wrist::intake));
      leftSpeed = 0.22;
      rightSpeed = 0.22;
    }
    if (m_timer.Get() >= 4.2_s && m_timer.Get() <= 5.15_s) {
      // pick up second note
      intake.SetControl(intakeOut);
    }
    if (m_timer.Get() >= 5.1_s && m_timer.Get() <= 6.05_s) {
      // move away from second note
      leftSpeed = -0.19;
      rightSpeed = -0.19;
    }
    if (m_timer.Get() >= 5.15_s && m_timer.Get() <= 5.65_s) {
      // stop rollers after second note picked up
      intake.SetControl(phx::controls::StaticBrake{});
    }
    if (m_timer.Get() >= 6.05_s && m_timer.Get() <= 6.9_s) {
      // turn towards amp to score second note
      leftSpeed = -0.2;
      rightSpeed = 0.2;
    }
    if (m_timer.Get() >= 6.9_s && m_timer.Get() <= 8.35_s) {
      // drive and score second note
      leftSpeed = -0.1;
      rightSpeed = -0.1;
      arm.SetControl(mmArm.WithPosition(arm::amp)); // untested ->.WithFeedForward(-0.2)); // should be dynamically calculated using arm angle
      wrist.SetControl(mmWrist.WithPosition(wrist::amp));
    }
    if (m_timer.Get() >= 8.35_s && m_timer.Get() <= 9.35_s) {
      // score second note
      intake.SetControl(intakeOut);
    }
    if (m_timer.Get() >= 9.35_s && m_timer.Get() <= 9.85_s) {
      // home position to end auto
      arm.SetControl(mmArm.WithPosition(arm::home)); // untested ->.WithFeedForward(-0.2)); // should be dynamically calculated using arm angle
      wrist.SetControl(mmWrist.WithPosition(wrist::home));
      intake.SetControl(phx::controls::StaticBrake{});
    }
  }
  else if (m_autoSelected == kAutoThreeNoteBlue) { // three note blue
    // no code
  }
  else if (m_autoSelected == kAutoFourNoteBlue) { // four note blue
    // no code
  }
  else if (m_autoSelected == kAutoNoMove) { // no move
    // no code
  }
  else { // shouldn't ever run, default to no movement
    // no code
  }
/*
      if (autonomousMode == 0) { // move forward
        if (m_timer.Get() >= 0_s && m_timer.Get() <= 2_s) {
          // drive forward
          rightSpeed = -0.1;
          leftSpeed = -0.1;
        }
      }
      else if (autonomousMode == 1) { // one note preload scorer
        if (m_timer.Get() >= 0_s && m_timer.Get() <= 0.6_s) {
          // drive forward
          rightSpeed = -0.1;
          leftSpeed = -0.1;
        }
        if (m_timer.Get() >= 0.6_s && m_timer.Get() <= 1.75_s) {
          // turn left
          leftSpeed = 0.2;
          rightSpeed = -0.2;
        }
        if (m_timer.Get() >= 1.75_s && m_timer.Get() <= 2.5_s) {
          // drive towards amp
          leftSpeed = -0.1;
          rightSpeed = -0.1;
          // raise arm to scoring position
          arm.SetControl(mmArm.WithPosition(arm::amp)); // untested ->.WithFeedForward(-0.2).WithFeedForward(0.2)); // should be dynamically calculated using arm angle
          wrist.SetControl(mmWrist.WithPosition(wrist::amp));
        }
        if (m_timer.Get() >= 2.6_s && m_timer.Get() <= 3.0_s) {
          // spin intake to score
          intake.SetControl(intakeOut);
        }
        if (m_timer.Get() >= 3.0_s && m_timer.Get() <= 5.5_s) {
          // stop intake after scored
          intake.SetControl(phx::controls::StaticBrake{});
          // lower arm and wrist to home position
          arm.SetControl(mmArm.WithPosition(arm::home));
          wrist.SetControl(mmWrist.WithPosition(wrist::home));
        }
      }
      else if (autonomousMode == 2) { // two note / preload + one from field
        if (m_timer.Get() >= 0_s && m_timer.Get() <= 0.6_s) {
          // drive forward
          rightSpeed = -0.1;
          leftSpeed = -0.1;
        }
        if (m_timer.Get() >= 0.6_s && m_timer.Get() <= 1.35_s) {
          // turn right
          leftSpeed = 0.21;
          rightSpeed = -0.21;
        }
        if (m_timer.Get() >= 1.35_s && m_timer.Get() <= 2.45_s) {
          // drive towards amp
          leftSpeed = -0.12;
          rightSpeed = -0.12;
          // raise arm to scoring position
          arm.SetControl(mmArm.WithPosition(arm::amp)); // untested ->.WithFeedForward(-0.2).WithFeedForward(0.2)); // should be dynamically calculated using arm angle
          wrist.SetControl(mmWrist.WithPosition(wrist::amp));
        }
        if (m_timer.Get() >= 2.45_s && m_timer.Get() <= 2.95_s) {
          // spin intake to score
          intake.SetControl(intakeOut);
        }
        if (m_timer.Get() >= 2.95_s && m_timer.Get() <= 3.45_s) {
          // stop intake after scored
          intake.SetControl(phx::controls::StaticBrake{});
          // lower arm and wrist to intake position
          arm.SetControl(mmArm.WithPosition(arm::intake));
          wrist.SetControl(mmWrist.WithPosition(wrist::intake));
          // drive towards second note
          leftSpeed = 0.27;
          rightSpeed = 0.27;
        }
        if (m_timer.Get() >= 3.45_s && m_timer.Get() <= 4.2_s) {
          // turn towards second note
          leftSpeed = 0.17;
          rightSpeed = -0.17;

        }
        if (m_timer.Get() >= 4.2_s && m_timer.Get() <= 5.1_s) {
          // pick up second note
          wrist.SetControl(mmWrist.WithPosition(wrist::intake));
          leftSpeed = 0.22;
          rightSpeed = 0.22;
        }
        if (m_timer.Get() >= 4.2_s && m_timer.Get() <= 5.15_s) {
          // pick up second note
          intake.SetControl(intakeOut);
        }
        if (m_timer.Get() >= 5.1_s && m_timer.Get() <= 6.05_s) {
          // move away from second note
          leftSpeed = -0.19;
          rightSpeed = -0.19;
        }
        if (m_timer.Get() >= 5.15_s && m_timer.Get() <= 5.65_s) {
          // stop rollers after second note picked up
          intake.SetControl(phx::controls::StaticBrake{});
        }
        if (m_timer.Get() >= 6.05_s && m_timer.Get() <= 6.9_s) {
          // turn towards amp to score second note
          leftSpeed = -0.2;
          rightSpeed = 0.2;
        }
        if (m_timer.Get() >= 6.9_s && m_timer.Get() <= 8.35_s) {
          // drive and score second note
          leftSpeed = -0.1;
          rightSpeed = -0.1;
          arm.SetControl(mmArm.WithPosition(arm::amp)); // untested ->.WithFeedForward(-0.2)); // should be dynamically calculated using arm angle
          wrist.SetControl(mmWrist.WithPosition(wrist::amp));
        }
        if (m_timer.Get() >= 8.35_s && m_timer.Get() <= 9.35_s) {
          // score second note
          intake.SetControl(intakeOut);
        }
        if (m_timer.Get() >= 9.35_s && m_timer.Get() <= 9.85_s) {
          // home position to end auto
          arm.SetControl(mmArm.WithPosition(arm::home)); // untested ->.WithFeedForward(-0.2)); // should be dynamically calculated using arm angle
          wrist.SetControl(mmWrist.WithPosition(wrist::home));
          intake.SetControl(phx::controls::StaticBrake{});
        }

      }
      else if (autonomousMode == 3) { // three note / preload + two from field
        if (m_timer.Get() >= 0_s && m_timer.Get() <= 0.6_s) {
          // drive forward
          rightSpeed = -0.1;
          leftSpeed = -0.1;
        }
        if (m_timer.Get() >= 0.6_s && m_timer.Get() <= 1.75_s) {
          // turn left
          leftSpeed = 0.1;
          rightSpeed = -0.1;
        }
        if (m_timer.Get() >= 1.75_s && m_timer.Get() <= 2.5_s) {
          // drive towards amp
          leftSpeed = -0.1;
          rightSpeed = -0.1;
          // raise arm to scoring position
          arm.SetControl(mmArm.WithPosition(arm::amp)); // untested ->.WithFeedForward(-0.2).WithFeedForward(0.2)); // should be dynamically calculated using arm angle
          wrist.SetControl(mmWrist.WithPosition(wrist::amp));
        }
        if (m_timer.Get() >= 2.6_s && m_timer.Get() <= 3.0_s) {
          // spin intake to score
          intake.SetControl(intakeOut);
        }
        if (m_timer.Get() >= 3.0_s && m_timer.Get() <= 5.5_s) {
          // stop intake after scored
          intake.SetControl(phx::controls::StaticBrake{});
          // lower arm and wrist to home position
          arm.SetControl(mmArm.WithPosition(arm::home));
          wrist.SetControl(mmWrist.WithPosition(wrist::home));
          // go to second note
          leftSpeed = 0.12;
          rightSpeed = -0.01;
        }
        if (m_timer.Get() >= 5.25_s && m_timer.Get() <= 6.25_s) {
          // pick up second note
          intake.SetControl(intakeOut);
        }
        if (m_timer.Get() >= 6.25_s && m_timer.Get() <= 6.5_s) {
          // stop roller after picked up second note
          intake.SetControl(phx::controls::StaticBrake{});
        }
        if (m_timer.Get() >= 6.0_s && m_timer.Get() <= 8.75_s) {
          // wrist and arm to home positon
          arm.SetControl(mmArm.WithPosition(arm::home));
          wrist.SetControl(mmWrist.WithPosition(wrist::home));
          // drive to amp
          leftSpeed = -0.11;
          rightSpeed = 0.01;
        }
        if (m_timer.Get() >= 8.75_s && m_timer.Get() <= 9.25_s) {
          // bring arm and wrist up to amp position
          arm.SetControl(mmArm.WithPosition(arm::amp));
          wrist.SetControl(mmWrist.WithPosition(wrist::amp));
        }
        if (m_timer.Get() >= 9.5_s && m_timer.Get() <= 10.0_s) {
          // shoot second note
          intake.SetControl(intakeOut);
        }
        if (m_timer.Get() >= 10.0_s && m_timer.Get() <= 10.5_s) {
          // stop intake rollers
          intake.SetControl(phx::controls::StaticBrake{});
          // bring wrist and arm to intake position
          arm.SetControl(mmArm.WithPosition(arm::intake)); // untested ->.WithFeedForward(-0.2)); // should be dynamically calculated using arm angle
          wrist.SetControl(mmWrist.WithPosition(wrist::intake));
        }
        if (m_timer.Get() >= 10.5_s && m_timer.Get() <= 12.25_s) {
          // go to third note
          leftSpeed = 0.275;
          rightSpeed = 0.15;
          // activate intake rollers
          intake.SetControl(intakeOut);
        }
        if (m_timer.Get() >= 12.25_s && m_timer.Get() <= 14.0_s) {
          // stop intake rollers
          intake.SetControl(phx::controls::StaticBrake{});
          // go back to amp
          leftSpeed = -0.275;
          rightSpeed = -0.15;
        }
        if (m_timer.Get() >= 14.0_s && m_timer.Get() <= 14.5_s) {
          // bring arm and wrist to amp position
          arm.SetControl(mmArm.WithPosition(arm::amp)); // untested ->.WithFeedForward(-0.2)); // should be dynamically calculated using arm angle
          wrist.SetControl(mmWrist.WithPosition(wrist::amp));
        }
        if (m_timer.Get() >= 14.5_s && m_timer.Get() <= 15.0_s) {
          // shoot third note
          intake.SetControl(intakeOut);
        }
      }
      else if (autonomousMode == 4) { // four note / don't use
        if (m_timer.Get() >= 0_s && m_timer.Get() <= 0.75_s) {
          leftSpeed = 0.22;
          rightSpeed = -0.22;
        }
        if (m_timer.Get() >= 0.75_s && m_timer.Get() <= 2_s) {
          leftSpeed = 0.16;
          rightSpeed = 0.16;
        }
      }
      else if (autonomousMode == 5) { // don't move
        // no code
      }
      else { // shouldn't ever run, default to no movement
        // no code
      }
*/

  /*
    if (m_timer.Get() >= 14.5_s && m_timer.Get() <= 15.0_s){
      intake.SetControl(phx::controls::StaticBrake{});
      arm.SetControl(mmArm.WithPosition(arm::home));
      wrist.SetControl(mmWrist.WithPosition(wrist::home));
    }
    */
  /*
  else {
    if (m_timer.Get() >= 0_s && m_timer.Get() <= 1.8_s) {
      // drive forward
      rightSpeed = -0.1;
      leftSpeed = -0.1;
      DEBUG_MSG("AutonomousPeriodic: speed = " << leftSpeed << " " << rightSpeed);
    }


    if (m_timer.Get() >= 4.5_s && m_timer.Get() <= 5.5_s){
    arm.SetControl(mmArm.WithPosition(arm::amp)); // untested ->.WithFeedForward(-0.2).WithFeedForward(0.2)); // should be dynamically calculated using arm angle
    wrist.SetControl(mmWrist.WithPosition(wrist::amp));
    }
    if (m_timer.Get() >= 3_s && m_timer.Get() <= 3.9_s){
      leftSpeed = 0.1;
      rightSpeed = -0.1;
    }
    if (m_timer.Get() >= 5.5_s && m_timer.Get() <= 7.25_s){
      leftSpeed = -0.05;
      rightSpeed = -0.05;
    }
    if (m_timer.Get() >= 8_s && m_timer.Get() <= 10_s){
      //intake.SetControl(intakeOut);
    }
    if (m_timer.Get() >= 11_s && m_timer.Get() <= 14_s){
      leftSpeed = 0.25;
      rightSpeed = -0.1;
      arm.SetControl(mmArm.WithPosition(arm::home)); // untested ->.WithFeedForward(-0.2)); // should be dynamically calculated using arm angle
      wrist.SetControl(mmWrist.WithPosition(wrist::home));
      intake.SetControl(phx::controls::StaticBrake{});

    }
  }
  */
  
  leftOut.Output = leftSpeed; // use this code to output speed variables into drive controls
  rightOut.Output = rightSpeed;
  leftDrive.SetControl(leftOut);
  rightDrive.SetControl(rightOut);
  
  /*
  if (!driveStaticBrake) {
    leftDrive.SetControl(leftOut);
    rightDrive.SetControl(rightOut);
  }
  else {
    leftDrive.SetControl(phx::controls::StaticBrake);
    rightDrive.SetControl(phx::controls::StaticBrake);
  }
  */
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