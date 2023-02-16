// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//Include statments
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
#include <cameraserver/CameraServer.h>

//gryo
#include "AHRS.h"
#include <frc/SerialPort.h>

class Robot : public frc::TimedRobot
{
public:
  Robot()
  {
    //Set up gyro
    navx = new AHRS(frc::SPI::Port::kMXP);

    m_timer.Start();

  }

  void RobotInit() override {
    frc::CameraServer::StartAutomaticCapture();
    //m_grabber.RobotInit();
    // Restore motor controller parameters to factory default
    m_ArmRetract.RestoreFactoryDefaults();
    m_ArmRotate.RestoreFactoryDefaults();

    // set PID coefficients
    m_ArmRotatePidController.SetP(kP);
    m_ArmRotatePidController.SetI(kI);
    m_ArmRotatePidController.SetD(kD);
    m_ArmRotatePidController.SetIZone(kIz);
    m_ArmRotatePidController.SetFF(kFF);
    m_ArmRotatePidController.SetOutputRange(kMinOutput, kMaxOutput);

    m_ArmRotatePidController.SetSmartMotionMaxVelocity(kMaxVel);
    m_ArmRotatePidController.SetSmartMotionMinOutputVelocity(kMinVel);
    m_ArmRotatePidController.SetSmartMotionMaxAccel(kMaxAcc);
    m_ArmRotatePidController.SetSmartMotionAllowedClosedLoopError(kAllErr);

    // display PID coefficients on SmartDashboard
    frc::SmartDashboard::PutNumber("P Gain", kP);
    frc::SmartDashboard::PutNumber("I Gain", kI);
    frc::SmartDashboard::PutNumber("D Gain", kD);
    frc::SmartDashboard::PutNumber("I Zone", kIz);
    frc::SmartDashboard::PutNumber("Feed Forward", kFF);
    frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
    frc::SmartDashboard::PutNumber("Min Output", kMinOutput);

    // display Smart Motion coefficients
    frc::SmartDashboard::PutNumber("Max Velocity", kMaxVel);
    frc::SmartDashboard::PutNumber("Min Velocity", kMinVel);
    frc::SmartDashboard::PutNumber("Max Acceleration", kMaxAcc);
    frc::SmartDashboard::PutNumber("Allowed Closed Loop Error", kAllErr);
    frc::SmartDashboard::PutNumber("Set Position", 0);
    frc::SmartDashboard::PutNumber("Set Velocity", 0);

    // button to toggle between velocity and smart motion modes
    frc::SmartDashboard::PutBoolean("Mode", true);

  }

  void AutonomousInit() override
  {
    //m_grabber.ModeInit();

  }

  void AutonomousPeriodic() override
  {


    // AUTOBALANCE CODE

    //xAxisRate is the speed of the robot
    double xAxisRate          = 0;
    //rollAngleDegrees is the angle the robot is at
    double rollAngleDegrees  = navx->GetRoll();

    //If the absolute value of the current angle of the robot is greater
    //the constant of what we consider is off balance, then change autoBalanceXMode to true;
    if ( !autoBalanceXMode &&
        (fabs(rollAngleDegrees) >=
        fabs(kOffBalanceThresholdDegrees))) {
      autoBalanceXMode = true;
    }
    //If the absolute value of the current angle of the robot is less
    //the constant of what we consider is on balance, then change autoBalanceXMode to false;
    else if ( autoBalanceXMode &&
        (fabs(rollAngleDegrees) <=
        fabs(kOnBalanceThresholdDegrees))) {
      autoBalanceXMode = false;
    }

    // If autoBalanceXMode is true, then calculate the speed of the robot (sin(Current angle * pi/180) * -1)
    // yes, we're actually using trig in real life for the first time ever. 
    if ( autoBalanceXMode ) {
      double rollAngleRadians = rollAngleDegrees * (M_PI / 180.0);
      xAxisRate = sin(rollAngleRadians) * -1;
    }

    // Set the drive speed of the robot to 60% of the speed we just calculated. This 60% may need to be changed again
    // once we test the autobalance code on the actual balance board. 
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
    //m_grabber.ModeInit();



  }

  void TeleopPeriodic() override
  {

    // check to see if autobalance should be turned on. 
    // mapped to button 4 on the drive controller
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
    //Rate limiter limits the acceleration of the turn speed to make it easier to control
    double turn = m_turnRateLimiter.Calculate(m_stick.GetTwist());

    if (fabs(speed) < deadband) {
      speed = 0.0;
    }

    // If the autobalance code is enabled, disable the drive speed from user inputs
    if (m_teleopBalance){
      speed = 0.0;
      turn = 0.0;

    }
    
    // The leaders are the front left and front right motors on the robot. Reads the input from the controller
    // and sets an output according to that value. The followers are the back left and back right motors on the robot.
    // They equal the exact output to the motor they are following. This is important because our gearboxes will never break
    // with this system. If a follower and leader were to go different directions, the robot won't be very happy. 
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
    // Autobalance code. Exactly the same as it is in the autonomous section.
    // Will hopefully be able to keep the robot balance while other robots drive onto the station
    // at the end of a match. Will disable user inputs. 
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

    //console output (delete the // to read the outputs of different things)
    
    //fmt::print("y={}\n", m_stick.GetY());
    //fmt::print("z={}\n", m_stick.GetTwist());

    //double yaw = navx->GetYaw();
    //fmt::print("pitch={}\n", navx->GetPitch());

    //m_grabber.RunPeriodic();
    fmt::print("Roll={}\n", navx->GetRoll());
    //fmt::print("ArmRotation={}\n", m_ArmRotateEncoder.GetPosition());

    //______________________________________________________________________________________
    //Arm System
    //Will be moved to a seperate file later. 
    // read PID coefficients from SmartDashboard
    double p = frc::SmartDashboard::GetNumber("P Gain", 0);
    double i = frc::SmartDashboard::GetNumber("I Gain", 0);
    double d = frc::SmartDashboard::GetNumber("D Gain", 0);
    double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
    double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
    double max = frc::SmartDashboard::GetNumber("Max Output", 0);
    double min = frc::SmartDashboard::GetNumber("Min Output", 0);
    double maxV = frc::SmartDashboard::GetNumber("Max Velocity", 0);
    double minV = frc::SmartDashboard::GetNumber("Min Velocity", 0);
    double maxA = frc::SmartDashboard::GetNumber("Max Acceleration", 0);
    double allE = frc::SmartDashboard::GetNumber("Allowed Closed Loop Error", 0);    

    if((p != kP))   { m_ArmRotatePidController.SetP(p); kP = p; }
    if((i != kI))   { m_ArmRotatePidController.SetI(i); kI = i; }
    if((d != kD))   { m_ArmRotatePidController.SetD(d); kD = d; }
    if((iz != kIz)) { m_ArmRotatePidController.SetIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_ArmRotatePidController.SetFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { m_ArmRotatePidController.SetOutputRange(min, max); kMinOutput = min; kMaxOutput = max; }
    if((maxV != kMaxVel)) { m_ArmRotatePidController.SetSmartMotionMaxVelocity(maxV); kMaxVel = maxV; }
    if((minV != kMinVel)) { m_ArmRotatePidController.SetSmartMotionMinOutputVelocity(minV); kMinVel = minV; }
    if((maxA != kMaxAcc)) { m_ArmRotatePidController.SetSmartMotionMaxAccel(maxA); kMaxAcc = maxA; }
    if((allE != kAllErr)) { m_ArmRotatePidController.SetSmartMotionAllowedClosedLoopError(allE); allE = kAllErr; }
  
    double SetPoint, ProcessVariable;
    bool mode = frc::SmartDashboard::GetBoolean("Mode", false);

    if(mode) {
      SetPoint = frc::SmartDashboard::GetNumber("Set Velocity", 0);
      m_ArmRotatePidController.SetReference(SetPoint, rev::CANSparkMax::ControlType::kVelocity);
      ProcessVariable = m_ArmRotateEncoder.GetVelocity();
    } else {
      SetPoint = frc::SmartDashboard::GetNumber("Set Position", 0);

          // If the Y button on the xbox controller is pressed, activate the rotation motor
    // on the arm. 
      if (m_xbox.GetYButtonPressed()){

        armFront = true;
        armCount += 1;
      
      }
      if (armCount % 3 == 1 && armFront == true){
        SetPoint = 13;
      } else if (armCount % 3 == 2 && armFront == true){
        SetPoint = 27;
      } else if (armCount % 3 == 0 && armFront == true){
        SetPoint = 40;
      }
      /**
       * As with other PID modes, Smart Motion is set by calling the
       * SetReference method on an existing pid object and setting
       * the control type to kSmartMotion
       */
      if (m_ArmRotateEncoder.GetPosition() > SetPoint + 0.1 || m_ArmRotateEncoder.GetPosition() < SetPoint - 0.1){
        m_ArmRotatePidController.SetReference(SetPoint, rev::CANSparkMax::ControlType::kSmartMotion);
      }
      ProcessVariable = m_ArmRotateEncoder.GetPosition();
    }    

    frc::SmartDashboard::PutNumber("Set Point", SetPoint);
    frc::SmartDashboard::PutNumber("Process Variable", ProcessVariable);
    frc::SmartDashboard::PutNumber("Output", m_ArmRotate.GetAppliedOutput());

//_________________________________________________________________________________________________________
//Grabber subsystem (Will be moved to a seperate file later)
    if (m_xbox.GetAButtonPressed())
    {
      m_runGrabberIntakeIn = !m_runGrabberIntakeIn;
    }
    if (m_xbox.GetBButtonPressed())
    {
      m_runGrabberIntakeOut = !m_runGrabberIntakeOut;
    }
    if (m_runGrabberIntakeIn)
    {

      m_runGrabberIntakeOut = false;
      m_GrabberIntake.Set(0.8);

    } else if (m_runGrabberIntakeOut)
    {
      m_runGrabberIntakeIn = false;
      m_GrabberIntake.Set(-0.8);
    } else {
      m_GrabberIntake.Set(0);
    }

  }


  void TestInit() override
  {
    m_testIndex = 0;
    m_stick.GetRawButtonPressed(testStartButton);
    m_stick.GetRawButtonPressed(testNextButton);
    m_runTest = false;
    //frc::SmartDashboard::PutNumber("AAindex", m_testIndex);
    //fmt::print("Switched to index {}  device id {}\n", m_testIndex, m_testDrives[m_testIndex]->GetDeviceId());

  }

  void TestPeriodic() override
  {

  //frc::SmartDashboard::PutNumber("AAindex", m_testIndex);
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

  // Slow drive starts as false, is enabled by pressing button 3
  bool m_slowDrive = false;
  // the balance mode in teleop starts as false, enabled by pressing button 4
  bool m_teleopBalance = false;

  //Talon drive motors
  TalonSRX m_leftLeader = {kDriveLeftLeader};
  TalonSRX m_leftFollower = {kDriveLeftFollower};
  TalonSRX m_rightLeader = {kDriveRightLeader};
  TalonSRX m_rightFollower = {kDriveRightFollower};

  //joystick USB port connection (assigned in driver station)
  // Make sure these are correctly assigned in the driver station, if they aren't the robot can't read any inputs
  frc::Joystick m_stick{0};
  frc::XboxController m_xbox{1};
  
  //Set up slew rate limiter
  frc::SlewRateLimiter<units::scalar> m_turnRateLimiter{1 / 1_s, 0};

  //exactly what you think, its a timer
  frc::Timer m_timer;

  //Set up gyro
  AHRS *navx;

  //Constants for autobalance, 10 degrees for off balance and 5 for on balance. 
  //I want to mess around with these constants at some point. 
  constexpr static const double kOffBalanceThresholdDegrees = 5.1f;
  constexpr static const double kOnBalanceThresholdDegrees  = 5.0f;
  bool autoBalanceXMode = false;
  bool autoBalanceYMode = false;

  //Grabber motors
  //TalonSRX m_GrabberIntake = {kGrabberIntakeID};
  //TalonSRX m_GrabberAngle = {kGrabberAngleID};
  rev::CANSparkMax m_GrabberIntake{kGrabberIntakeID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_GrabberAngle{kGrabberAngleID, rev::CANSparkMax::MotorType::kBrushless};

  //Grabber constants
  bool m_runGrabberIntakeIn = false;
  bool m_runGrabberIntakeOut = false;
  int grabberCount = 1;


  //Grabber subsystem (currently broken yay :D)
  //GrabberSubsystem m_grabber{m_GrabberIntake, m_GrabberAngle, m_xbox};

  //Arm NEO motors
  rev::CANSparkMax m_ArmRotate{kArmRotateID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_ArmRetract{kArmRetractID, rev::CANSparkMax::MotorType::kBrushless};

  rev::SparkMaxRelativeEncoder m_ArmRotateEncoder = m_ArmRotate.GetEncoder();
  rev::SparkMaxPIDController m_ArmRotatePidController = m_ArmRotate.GetPIDController();

  // default PID coefficients
  double kP = 5e-5, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000156, kMaxOutput = 1, kMinOutput = -1;

  // default smart motion coefficients
  double kMaxVel = 2000, kMinVel = 0, kMaxAcc = 1500, kAllErr = 0;

  // motor max RPM
  const double MaxRPM = 5700;  

  //Arm constants
  bool armFront = false;
  int armCount = 0;





  // Allow the robot to access the data from the camera. 
  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
  double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
  double targetArea = table->GetNumber("ta",0.0);
  double targetSkew = table->GetNumber("ts",0.0);

  // independent motor test
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
