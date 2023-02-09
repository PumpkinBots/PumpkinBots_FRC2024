// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <fstream>
#include <string>
#include <sstream>

#include <fmt/core.h>
#include <frc/Filesystem.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>


#include "rev/CANSparkMax.h"
#include "Constants.h"

#include "subsystems/GrabberSubsystem.h"

#include "networktables/NetworkTable.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"

#include <units/dimensionless.h>
#include <frc/filter/SlewRateLimiter.h>

#include "ctre/Phoenix.h"

//gryo
#include "AHRS.h"
#include <frc/SerialPort.h>

//using namespace frc;

class Robot : public frc::TimedRobot
{
public:
  Robot()
  {
    navx = new AHRS(frc::SPI::Port::kMXP);

    //navx = new AHRS(SerialPort::kMXP);

    //m_robotDrive.SetExpiration(100_ms);
    m_timer.Start();
    /*
    m_testDrives.push_back(&m_leftLeader);
    m_testDrives.push_back(&m_leftFollower);
    m_testDrives.push_back(&m_rightLeader);
    m_testDrives.push_back(&m_rightFollower);
  */
  }

  void RobotInit() override {

    //m_grabber.RobotInit();
    m_ArmRetract.RestoreFactoryDefaults();
    m_ArmRotate.RestoreFactoryDefaults();




  }

  void AutonomousInit() override
  {
    //m_grabber.ModeInit();

  }

  void AutonomousPeriodic() override
  {

    double xAxisRate          = 0;
    double rollAngleDegrees  = navx->GetRoll();

    if ( !autoBalanceXMode &&
        (fabs(rollAngleDegrees) >=
        fabs(kOffBalanceThresholdDegrees))) {
      autoBalanceXMode = true;
    }
    else if ( autoBalanceXMode &&
        (fabs(rollAngleDegrees) <=
        fabs(kOnBalanceThresholdDegrees))) {
      autoBalanceXMode = false;
    }

    if ( autoBalanceXMode ) {
      double rollAngleRadians = rollAngleDegrees * (M_PI / 180.0);
      xAxisRate = sin(rollAngleRadians) * -1;
    }

    m_leftLeader.Set(ControlMode::PercentOutput, (0.6)*xAxisRate);
    m_rightLeader.Set(ControlMode::PercentOutput,-(0.6)*xAxisRate);
    m_leftFollower.Follow(m_leftLeader);
    m_rightFollower.Follow(m_rightLeader);    

    //fmt::print("speed={}\n", xAxisRate);

  }

  void TeleopInit() override 
  {
    //m_robotDrive.StopMotor();
    m_turnRateLimiter.Reset(0);
    m_grabber.ModeInit();

  }

  void TeleopPeriodic() override
  {
    //_______________________________________________________
    //Autobalance (teleop version) 
    if (m_stick.GetRawButtonPressed(4)){
      m_teleopBalance = !m_teleopBalance;
    }


    //______________________________________________________________________________
    //DRIVE SYSTEM

    //Button three on the joystick toggles "slow drive" mode.
    //In this mode, the robot's drive and turn speed are limited to 30%.
    if (m_stick.GetRawButtonPressed(3)){
      m_slowDrive = !m_slowDrive;
    }

    //deadband value of 0.05, throw out any inputs less than that value
    const double deadband = 0.05;
    double speed = m_stick.GetY();
    double turn = m_turnRateLimiter.Calculate(m_stick.GetTwist());

    if (fabs(speed) < deadband) {
      speed = 0.0;
    }

    if (m_teleopBalance){
      speed = 0.0;
      turn = 0.0;

    }
    
    if (m_slowDrive){
      m_leftLeader.Set(ControlMode::PercentOutput, -(0.3)*speed, DemandType::DemandType_ArbitraryFeedForward, (0.3)*turn);
      m_rightLeader.Set(ControlMode::PercentOutput, (0.3)*speed, DemandType::DemandType_ArbitraryFeedForward, (0.3)*turn);
      m_leftFollower.Follow(m_leftLeader);
      m_rightFollower.Follow(m_rightLeader);
    } else {
      m_leftLeader.Set(ControlMode::PercentOutput, -speed, DemandType::DemandType_ArbitraryFeedForward, turn);
      m_rightLeader.Set(ControlMode::PercentOutput, speed, DemandType::DemandType_ArbitraryFeedForward, turn);
      m_leftFollower.Follow(m_leftLeader);
      m_rightFollower.Follow(m_rightLeader);
    }
    //_____________________________________________________________________________
    //Arm System
    if (m_xbox.GetYButtonPressed()){

      armFront = !armFront;
    
    }
    if (armFront){
      //m_ArmRetract.GetEncoder().SetPosition(21);
    } else {
      //m_ArmRetract.GetEncoder().SetPosition(0);

    }

    if (m_teleopBalance){

          double xAxisRate          = 0;
          double rollAngleDegrees  = navx->GetRoll();

          if ( !autoBalanceXMode &&
              (fabs(rollAngleDegrees) >=
              fabs(kOffBalanceThresholdDegrees))) {
            autoBalanceXMode = true;
          }
          else if ( autoBalanceXMode &&
              (fabs(rollAngleDegrees) <=
              fabs(kOnBalanceThresholdDegrees))) {
            autoBalanceXMode = false;
          }

          if ( autoBalanceXMode ) {
            double rollAngleRadians = rollAngleDegrees * (M_PI / 180.0);
            xAxisRate = sin(rollAngleRadians) * -1;
          }

          m_leftLeader.Set(ControlMode::PercentOutput, (0.6)*xAxisRate);
          m_rightLeader.Set(ControlMode::PercentOutput,-(0.6)*xAxisRate);
          m_leftFollower.Follow(m_leftLeader);
          m_rightFollower.Follow(m_rightLeader);

    }

    //console output
    
    //fmt::print("y={}\n", m_stick.GetY());
    //fmt::print("z={}\n", m_stick.GetTwist());

    //double yaw = navx->GetYaw();
    //fmt::print("pitch={}\n", navx->GetPitch());

    //m_grabber.RunPeriodic();
    fmt::print("Roll={}\n", navx->GetRoll());

  
  }


  void TestInit() override
  {
    // Disable to drive motors in Test mode so that the robot stays on the bench.
    //m_robotDrive.StopMotor();


    m_testIndex = 0;
    m_stick.GetRawButtonPressed(testStartButton);
    m_stick.GetRawButtonPressed(testNextButton);
    m_runTest = false;
    frc::SmartDashboard::PutNumber("AAindex", m_testIndex);
    fmt::print("Switched to index {}  device id {}\n", m_testIndex, m_testDrives[m_testIndex]->GetDeviceId());

  }

  void TestPeriodic() override
  {

  frc::SmartDashboard::PutNumber("AAindex", m_testIndex);
  if (m_stick.GetRawButtonPressed(testStartButton)) {
    m_runTest = !m_runTest;
  }
  if (m_runTest) {
    //m_testDrives[m_testIndex]->Set(0.5);
    m_testDrives[m_testIndex]->Set(m_stick.GetThrottle());
  } else {
    m_testDrives[m_testIndex]->Set(0);
    if (m_stick.GetRawButtonPressed(testNextButton)) {
      m_testIndex++;
      if (m_testIndex > 3) { m_testIndex = 0;}
      fmt::print("Switched to index {}  device id {}\n", m_testIndex, m_testDrives[m_testIndex]->GetDeviceId());
    }
  }

  }

private:

  std::string m_buildVersion;
  bool m_slowDrive = false;
  bool m_teleopBalance = false;
  //Talon drive motors
  TalonSRX m_leftLeader = {kDriveLeftLeader};
  TalonSRX m_leftFollower = {kDriveLeftFollower};
  TalonSRX m_rightLeader = {kDriveRightLeader};
  TalonSRX m_rightFollower = {kDriveRightFollower};
  //frc::DifferentialDrive m_robotDrive{m_leftLeader, m_rightLeader};

  //joystick USB port connection (assigned in driver station)
  frc::Joystick m_stick{0};
  frc::XboxController m_xbox{1};
  
  frc::SlewRateLimiter<units::scalar> m_turnRateLimiter{1 / 1_s, 0};
  frc::Timer m_timer;

  AHRS *navx;

  constexpr static const double kOffBalanceThresholdDegrees = 10.0f;
  constexpr static const double kOnBalanceThresholdDegrees  = 5.0f;
  bool autoBalanceXMode = false;
  bool autoBalanceYMode = false;

  TalonSRX m_GrabberIntake = {kGrabberIntakeID};
  TalonSRX m_GrabberAngle = {kGrabberAngleID};

  GrabberSubsystem m_grabber{m_GrabberIntake, m_GrabberAngle, m_xbox};

  rev::CANSparkMax m_ArmRotate{kArmRotateID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_ArmRetract{kArmRetractID, rev::CANSparkMax::MotorType::kBrushless};

  bool armFront = false;



  // Allow the robot to access the data from the camera. 
  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
  double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
  double targetArea = table->GetNumber("ta",0.0);
  double targetSkew = table->GetNumber("ts",0.0);

  // Silly independent motor test
  int m_testIndex = 0;
  int testStartButton = 7;
  int testNextButton = 8;
  bool m_runTest = false;
  std::vector<rev::CANSparkMax*> m_testDrives;
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif