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
  }

  void RobotInit() override {
    // Initialize drive motors
    m_leftLeader.RestoreFactoryDefaults();
    m_leftFollower.RestoreFactoryDefaults();
    m_rightLeader.RestoreFactoryDefaults();
    m_rightFollower.RestoreFactoryDefaults();
    */
    // Set followers
    //m_leftFollower.Follow(m_leftLeader);
    //m_rightFollower.Follow(m_rightLeader);

    /*
    // Read the build version from the deploy directory.
    // https://docs.wpilib.org/en/stable/docs/software/advanced-gradlerio/deploy-git-data.html
    std::string deployDir = frc::filesystem::GetDeployDirectory();
    std::ifstream branchFile(deployDir + "/branch.txt");
    std::string branchStr;
    branchFile >> branchStr;  // This should suck up the whole file into the string.
    fmt::print("Branch: {}\n", branchStr.c_str());  // This prints to the console.
    std::ifstream commitFile(deployDir + "/commit.txt");
    std::string commitStr;
    commitFile >> commitStr;  // This should suck up the whole file into the string.
    fmt::print("Commit: {}\n", commitStr.c_str());

    // Format the displayed version using an sstream.
    std::ostringstream buildVersStream;
    buildVersStream << "Branch: " << branchStr << " Commit: " << commitStr;
    m_buildVersion = buildVersStream.str();

    fmt::print("Formated m_buildVersion: |{}|\n",  m_buildVersion);
    frc::SmartDashboard::PutString("Robot Code Version", m_buildVersion);

    std::string gitVersion = GetRobotVersion();
    fmt::print("Version: {}\n", gitVersion);
    frc::SmartDashboard::PutString("Robot Code Git Version", gitVersion);

    std::string buildInfo = GetBuildInfo();
    fmt::print("Build Info: {}\n", buildInfo);
    frc::SmartDashboard::PutString("Robot Build Info", buildInfo);
    */

   
  }

  void AutonomousInit() override
  {
    //m_robotDrive.StopMotor();

    m_timer.Reset();
    m_timer.Start();
  }

  void AutonomousPeriodic() override
  {
    double xAxisRate          = m_stick.GetX();
    double pitchAngleDegrees  = navx->GetPitch();

    if ( !autoBalanceXMode &&
        (fabs(pitchAngleDegrees) >=
        fabs(kOffBalanceThresholdDegrees))) {
      autoBalanceXMode = true;
    }
    else if ( autoBalanceXMode &&
        (fabs(pitchAngleDegrees) <=
        fabs(kOnBalanceThresholdDegrees))) {
      autoBalanceXMode = false;
    }

    if ( autoBalanceXMode ) {
      double pitchAngleRadians = pitchAngleDegrees * (M_PI / 180.0);
      xAxisRate = sin(pitchAngleRadians) * -1;
    }

    m_leftLeader.Set(ControlMode::PercentOutput, -xAxisRate);
    m_rightLeader.Set(ControlMode::PercentOutput, xAxisRate);
    m_leftFollower.Follow(m_leftLeader);
    m_rightFollower.Follow(m_rightLeader);    

    fmt::print("speed={}\n", xAxisRate);

  }

  void TeleopInit() override 
  {
    //m_robotDrive.StopMotor();
    m_turnRateLimiter.Reset(0);

  }

  void TeleopPeriodic() override
  {
    // Y-axis is negative pushed forward, and now the drive forward
    // is also negative. However invert the twist input.

    //These two lines are for different motors, keep them commented out.
    //m_robotDrive.ArcadeDrive(m_stick.GetY(), m_turnRateLimiter.Calculate(-m_stick.GetTwist()), true);
    //m_robotDrive.ArcadeDrive(m_stick.GetY(), (0.5)*-m_stick.GetTwist(), false);
    
    //Button three on the joystick toggles "slow drive" mode.
    //In this mode, the robot's drive and turn speed are limited to 30%.
    if (m_stick.GetRawButtonPressed(3)){
      m_slowDrive = !m_slowDrive;
    }

    const double deadband = 0.05;
    double speed = m_stick.GetY();
    double turn = m_turnRateLimiter.Calculate(m_stick.GetTwist());

    if (fabs(speed) < deadband) {
      speed = 0.0;
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


    //console output
    
    //fmt::print("y={}\n", m_stick.GetY());
    //fmt::print("z={}\n", m_stick.GetTwist());

    //double yaw = navx->GetYaw();
    //fmt::print("pitch={}\n", navx->GetPitch());
  
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