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
  wristConf.MotorOutput.Inverted = false; // verified

  // arm and wrist into break mode
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
  armSlot0Conf.kP = 0; // A position error of 2.5 rotations results in 12 V output
  armSlot0Conf.kI = 0; // no output for integrated error
  armSlot0Conf.kD = 0; // A velocity error of 1 rps results in 0.1 V output

  auto& mmArmConf = armConf.MotionMagic;
  mmArmConf.MotionMagicCruiseVelocity = 1;
  mmArmConf.MotionMagicAcceleration = 1;
  mmArmConf.MotionMagicJerk = 200;
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
  mmWristConf.MotionMagicCruiseVelocity = 1;
  mmWristConf.MotionMagicAcceleration = 1;
  mmWristConf.MotionMagicJerk = 200;
  wrist.GetConfigurator().Apply(wristConf);

  /* assume start in home position */
  arm.SetPosition(arm::home);
  wrist.SetPosition(arm::home);

  /* Intake configuration */
  phx::configs::TalonFXConfiguration intakeConf{};
 
  /* Set rotation direction for the arm and wrist */
  /**
   * FIXME: these are RANDOMLY chosen - review literature and cad to verify
  */
  intakeConf.MotorOutput.Inverted = false; // primary intake at left when facing the intake mechanism
  //intakeConf.MotorOutput.NeutralMode = phx::signals::NeutralModeValue::Brake;

  intake.GetConfigurator().Apply(intakeConf);
  intakeFollower.GetConfigurator().Apply(intakeConf);
  
  /* Set up followers to follow leaders and retain the leaders' inversion settings */
  intakeFollower.SetControl(phx::controls::Follower{intake.GetDeviceID(), false});

  // setup motor braking - 
  leftDrive.SetControl(phx::controls::NeutralOut{});
  rightDrive.SetControl(phx::controls::NeutralOut{});
  arm.SetControl(phx::controls::StaticBrake{});
  wrist.SetControl(phx::controls::StaticBrake{});
  intake.SetControl(phx::controls::NeutralOut{});

  intakeOut.Output = intake::intakeOut;

  // offsets for wrist and arm
  m_wristStartPos = wrist.GetPosition().GetValueAsDouble();
  m_armStartPos = arm.GetPosition().GetValueAsDouble();
  std::cout << "m_wristStartPos: " << m_wristStartPos << " m_armStartPos" << m_armStartPos << std::endl;
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
    std::cout << "limited maxSpeed: " << slowDrive;
  }
  const double maxSpeed = slowDrive ? 0.3 : 1.0;

  /**
   * REVERSE DRIVE
   * Button two on the joystick toggles the reverse drive mode
   * FIXME: move this button to something more user friendly like the thumb button
  */
  if (joystick.GetRawButtonPressed(2)) {
    reverseDrive = !reverseDrive;
    std::cout << "reverse drive: " << reverseDrive;
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
  leftOut.Output = maxSpeed * (speed + speedTurn);
  rightOut.Output = maxSpeed * (speed - speedTurn); 

  if (reverseDrive) {
    leftDrive.SetInverted(-leftDrive.GetInverted());
    rightDrive.SetInverted(-rightDrive.GetInverted());
    leftOut.Output = - leftOut.Output;
    rightOut.Output = - rightOut.Output;
  }

  leftDrive.SetControl(leftOut);
  rightDrive.SetControl(rightOut);

  // Arm, wrist and intake control
  double armPosition = arm.GetPosition().GetValueAsDouble();
  double wristPosition = wrist.GetPosition().GetValueAsDouble();
  double inTakePosition = intake.GetPosition().GetValueAsDouble();
  //armMoving = arm.GetVelocity().GetValueAsDouble() != 0.0 ? true : false;
  //wristMoving = wrist.GetVelocity().GetValueAsDouble() != 0.0 ? true : false;
  //noteDetected = noteSensor.Get();
  
  if (m_printCount++ > 50) {
    m_printCount = 0;
    std::cout << "Arm position: " << armPosition << " velocity: " << arm.GetVelocity().GetValueAsDouble() << "\n";
    std::cout << "Wrist position: " << wristPosition << " velocity: " <<  wrist.GetVelocity().GetValueAsDouble() << "\n";
    std::cout << "Intake position: " << inTakePosition << " velocity: " << intake.GetVelocity().GetValueAsDouble() << "\n";
    std::cout << "noteDetected: " << noteDetected << "\n";
  }
 

  // inTake Control - Y is Up, A is down, else stop. If we have a noteDetected also stop
  if (xbox.GetYButton()) {
      intake.SetControl(intakeOut);
  } else {
    intake.SetControl(m_brake);
  }

  const double wristMin = -0 + m_wristStartPos;
  const double wristMax = 14.0 + m_wristStartPos;
  const double wristStep = 0.1;
  // Wrist Control - Left Bumper is down, Right Bumper is up, else stop
  if ((xbox.GetLeftBumper()) && (wristPosition > wristMin)) {
    wrist.SetControl(mmWrist.WithPosition((wristPosition-wristStep)*1_tr).WithSlot(0));
  } else if ((xbox.GetRightBumper()) && (wristPosition < wristMax)) {
    wrist.SetControl(mmWrist.WithPosition((wristPosition+wristStep)*1_tr).WithSlot(0));
  } else {
    wrist.SetControl(m_brake);
  }

  const double armMin = -5.0 + m_armStartPos;
  const double armMax = 70.5 + m_armStartPos;
  const double armStep = 1.0;
  // Arm Control - Left Trigger is down, right trigger is up, else stop
  if ((xbox.GetLeftTriggerAxis()) && (armPosition > armMin)){
    arm.SetControl(mmArm.WithPosition((armPosition-armStep)*1_tr).WithSlot(0));
  } else if ((xbox.GetRightTriggerAxis()) && (armPosition < armMax)) {
    arm.SetControl(mmArm.WithPosition((armPosition+armStep)*1_tr).WithSlot(0));
  } else {
    arm.SetControl(m_brake);
  }
  
  // Manual mode, slow arm and wrist speed while holding start button and x and y
  if (xbox.GetStartButton()) {
    slowDownWereTesting = 0.1;
    armSpeed = (fabs(xbox.GetRightY()) > deadband) ? xbox.GetRightY() : 0.0;
    wristSpeed = (fabs(xbox.GetLeftY()) > deadband) ? xbox.GetLeftY() : 0.0;
    armOut.Output = slowDownWereTesting * armSpeed;
    wristOut.Output = - slowDownWereTesting * wristSpeed;
    arm.SetControl(armOut);
    wrist.SetControl(wristOut);
  }
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif