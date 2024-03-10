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
//#ifdef ROBOT_CAMERA
  frc::CameraServer::StartAutomaticCapture();
//#endif

  /**
   * DRIVE MOTOR CONFIGURATION
  */
  
  /* Drive configuration */
  phx::configs::TalonFXConfiguration leftConf{};
  phx::configs::TalonFXConfiguration rightConf{};

  /* Controls which end of the robot is front - these should always be set in opposition */
  leftConf.MotorOutput.Inverted = false;
  rightConf.MotorOutput.Inverted = true;

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
  wristConf.MotorOutput.Inverted = true; // verified

  // arm and wrist into brake mode
  armConf.MotorOutput.NeutralMode = phx::signals::NeutralModeValue::Brake;
  wristConf.MotorOutput.NeutralMode = phx::signals::NeutralModeValue::Brake;

  // setup sensor to shaft ratio to 1 for now
  phx::configs::FeedbackConfigs &armfdb = armConf.Feedback;
  armfdb.SensorToMechanismRatio = 1;
  phx::configs::FeedbackConfigs &wristfdb = wristConf.Feedback;
  wristfdb.SensorToMechanismRatio = 1;

  /**
   * slot0 defines the PID characteristics of MotionMagic
   * FIXME: characterize this properly - this is copy-pasta crap
  */
  auto& armSlot0Conf = armConf.Slot0;
  armSlot0Conf.kS = 0.05; // Add 0.25 V output to overcome static friction
  armSlot0Conf.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
  armSlot0Conf.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
  armSlot0Conf.kP = 0.0; // A position error of 2.5 rotations results in 12 V output
  armSlot0Conf.kI = 0.0; // no output for integrated error
  armSlot0Conf.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

  auto& mmArmConf = armConf.MotionMagic;
  mmArmConf.MotionMagicCruiseVelocity = 2;
  mmArmConf.MotionMagicAcceleration = 2;
  mmArmConf.MotionMagicJerk = 50;
  arm.GetConfigurator().Apply(armConf);
  armFollower.GetConfigurator().Apply(armConf);

  armFollower.SetControl(phx::controls::Follower{arm.GetDeviceID(), true}); // inverted rotation

  auto& wristSlot0Conf = wristConf.Slot0;
  wristSlot0Conf.kS = 0.05; // Add 0.25 V output to overcome static friction
  wristSlot0Conf.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
  wristSlot0Conf.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
  wristSlot0Conf.kP = 0.0; // A position error of 2.5 rotations results in 12 V output
  wristSlot0Conf.kI = 0.0; // no output for integrated error
  wristSlot0Conf.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

  auto& mmWristConf = wristConf.MotionMagic;
  mmWristConf.MotionMagicCruiseVelocity = 10.0;
  mmWristConf.MotionMagicAcceleration = 10.0;
  mmWristConf.MotionMagicJerk = 50;
  wrist.GetConfigurator().Apply(wristConf);

  // set to position 0 so we can track position accurately
  arm.SetPosition(0_tr);
  wrist.SetPosition(0_tr);

 // offsets for wrist and arm, as precision 0 is not quite 0.
  m_wristStartPos = wrist.GetPosition().GetValueAsDouble();
  m_armStartPos = arm.GetPosition().GetValueAsDouble();
  DEBUG_MSG("m_wristStartPos: " << m_wristStartPos << " m_armStartPos" << m_armStartPos);

  /* Intake configuration */
  phx::configs::TalonFXConfiguration intakeConf{};
  intakeConf.MotorOutput.Inverted = false; // primary intake at left when facing the intake mechanism
  //intakeConf.MotorOutput.NeutralMode = phx::signals::NeutralModeValue::Brake;

  intake.GetConfigurator().Apply(intakeConf);
  intakeFollower.GetConfigurator().Apply(intakeConf);
  
  /* Set up followers to follow leaders and retain the leaders' inversion settings */
  intakeFollower.SetControl(phx::controls::Follower{intake.GetDeviceID(), false});

  // FIXME: setup motor braking but don't know how to verify
  leftDrive.SetControl(phx::controls::NeutralOut{});
  rightDrive.SetControl(phx::controls::NeutralOut{});
  arm.SetControl(phx::controls::StaticBrake{});
  wrist.SetControl(phx::controls::StaticBrake{});
  intake.SetControl(phx::controls::NeutralOut{});
  intakeOut.Output = intake::intakeOut;
 }

// FIXME - this should be delayed by 3 seconds based on recommendations with robots leaving brake mode too early and moving in competition
void Robot::DisabledPeriodic() {
  leftDrive.SetControl(phx::controls::NeutralOut{});
  rightDrive.SetControl(phx::controls::NeutralOut{});
  arm.SetControl(phx::controls::NeutralOut{});
  wrist.SetControl(phx::controls::NeutralOut{});
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
    DEBUG_MSG("limited maxSpeed: " << slowDrive);
  }
  const double maxSpeed = slowDrive ? 0.3 : 1.0;

  /**
   * REVERSE DRIVE
   * Button two on the joystick toggles the reverse drive mode
   * FIXME: move this button to something more user friendly like the thumb button
  */
  if (joystick.GetRawButtonPressed(2)) {
    reverseDrive = !reverseDrive;
    DEBUG_MSG("reverse drive: " << reverseDrive);
  }

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
  speedTurn = -speedTurn;
  /**
   * DRIVE OUTPUT (speed + speedTurn)
   * The existing calculation will result in even less aggressive turning at max speed due to effectively saturating the leading drive motor.
   * It might be better to cap speed + speedTurn at 1/-1
   * or it might be better to leave as-is as it further limits turning at speed
   * FIXME: explore alternate implementations of turning + speed to make it as smooth and predictable as possible for the driver
  */
  leftOut.Output = maxSpeed * (speed - speedTurn);
  rightOut.Output = maxSpeed * (speed + speedTurn); 

  if (reverseDrive) {
    leftDrive.SetInverted(!leftDrive.GetInverted());
    rightDrive.SetInverted(!rightDrive.GetInverted());
    leftOut.Output = - leftOut.Output;
    rightOut.Output = - rightOut.Output;
  }

  leftDrive.SetControl(leftOut);
  rightDrive.SetControl(rightOut);

  // Arm, wrist and intake control
  double armPosition = arm.GetPosition().GetValueAsDouble();
  double wristPosition = wrist.GetPosition().GetValueAsDouble();
  //armMoving = arm.GetVelocity().GetValueAsDouble() != 0.0 ? true : false;
  //wristMoving = wrist.GetVelocity().GetValueAsDouble() != 0.0 ? true : false;

  // FIXME : noteDetected not being used yet
  noteDetected = noteSensor.Get();
  
  if (m_printCount++ > 200) {
    m_printCount = 0;
    DEBUG_MSG("Arm position: " << armPosition << " velocity: " << arm.GetVelocity().GetValueAsDouble());
    DEBUG_MSG("Wrist position: " << wristPosition << " velocity: " <<  wrist.GetVelocity().GetValueAsDouble());
    DEBUG_MSG("noteDetected: " << noteDetected << "\n");
  }
 

  // inTake Control - Y is Spin, else stop
  if (xbox.GetYButton()) {
      intake.SetControl(intakeOut);
  } else if (xbox.GetAButton()) {
      intake.SetInverted(!intake.GetInverted());
      intake.SetControl(intakeOut);
      intake.SetInverted(!intake.GetInverted());
  } else {
    intake.SetControl(m_brake);
  }

  const double wristMin = 0.0 + m_wristStartPos;
  const double wristMax = 75.0 + m_wristStartPos;
  const double wristStep = 1.0;
  // Wrist Control - Left Bumper is down, Right Bumper is up, else stop
  if ((xbox.GetLeftBumper()) && (wristPosition > wristMin)) {
    wrist.SetControl(mmWrist.WithPosition((wristPosition-wristStep)*1_tr).WithSlot(0));
  } else if ((xbox.GetRightBumper()) && (wristPosition < wristMax)) {
    wrist.SetControl(mmWrist.WithPosition((wristPosition+wristStep)*1_tr).WithSlot(0));
  } else {
    wrist.SetControl(m_brake);
  }

  const double armMin = 0.0 + m_armStartPos;
  const double armMax = 63 + m_armStartPos;
  const double armStep = 2.0;
  // Arm Control - Left Trigger is down, right trigger is up, else stop
  if ((xbox.GetRightTriggerAxis()) && (armPosition > armMin)){
    arm.SetControl(mmArm.WithPosition((armPosition-armStep)*1_tr).WithSlot(0));
  } else if ((xbox.GetLeftTriggerAxis()) && (armPosition < armMax)) {
    arm.SetControl(mmArm.WithPosition((armPosition+armStep)*1_tr).WithSlot(0));
  } else {
    arm.SetControl(m_brake);
  }
  
  // Manual mode, cycle through increasing speed output
  if (xbox.GetBButtonPressed()) {
    m_speedCycle = ((m_speedCycle % 10) + 1) / 10;
    double armSpeed = arm.GetVelocity().GetValueAsDouble() * m_speedCycle;
    double wristSpeed = wrist.GetVelocity().GetValueAsDouble() * m_speedCycle;
    armOut.Output = armSpeed;
    wristOut.Output = wristSpeed;
    DEBUG_MSG("XboxBButton: Setting arm speed to:" << armSpeed << "wrist speed to: "  << wristSpeed << " m_speedCycle: " << m_speedCycle );
    arm.SetControl(armOut);
    wrist.SetControl(wristOut);
  }

  // Reset postion for arm and write due to slop in drive chain mechanisms
  if (xbox.GetXButton()) {
    DEBUG_MSG("XboxAButton: Recalibrating arm and wrist to 0, mechanism should be at home position!!!");
    arm.SetPosition(0_tr);
    wrist.SetPosition(0_tr);
  }
}

void Robot::AutonomousInit() {
    //Set timer to zero and start counting
    m_timer.Reset();
    m_timer.Start();
    //autoMode = frc::SmartDashboard::GetNumber("Auto mode", 0);
}

void Robot::AutonomousPeriodic() {
  double speed = 0.0;

  // we can vary speed or time to increase distance, speed of 0.1 is a safe speed
  if (m_timer.Get() >= 0_s && m_timer.Get() <= 2_s) {
    // drive forward
    speed = 0.1;
    DEBUG_MSG("AutonomousPeriodic: speed=" << speed);
  } 
  else {
    speed = 0.0;
  }
  
  leftOut.Output = speed;
  rightOut.Output = speed; 

  leftDrive.SetControl(leftOut);
  rightDrive.SetControl(rightOut);
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif