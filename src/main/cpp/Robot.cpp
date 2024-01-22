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

    frc::SmartDashboard::PutNumber("Auto mode", 0);

    frc::CameraServer::StartAutomaticCapture();
    frc::CameraServer::StartAutomaticCapture();

    //m_grabber.RobotInit();
    // Restore motor controller parameters to factory default

    m_GrabberAngle.RestoreFactoryDefaults();
    m_GrabberIntake.RestoreFactoryDefaults();


    m_leftLeader.ConfigFactoryDefault();
    m_leftFollower.ConfigFactoryDefault();
    m_rightLeader.ConfigFactoryDefault();
    m_rightFollower.ConfigFactoryDefault();

    m_rightLeader.SetSelectedSensorPosition(0, 0, 10);


    //m_rightLeader.ConfigFactoryDefault();
    int absolutePositionRight = m_rightLeader.GetSensorCollection().GetPulseWidthPosition();
/* use the low level API to set the quad encoder signal */
		m_rightLeader.SetSelectedSensorPosition(absolutePositionRight, kPIDLoopIdx,
				kTimeoutMs);
    m_rightLeader.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
		m_rightLeader.SetSensorPhase(true);

    m_rightLeader.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
    m_rightLeader.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

		/* set the peak and nominal outputs, 12V means full */
    m_rightLeader.ConfigNominalOutputForward(0, 10);
    m_rightLeader.ConfigNominalOutputReverse(0, 10);
    m_rightLeader.ConfigPeakOutputForward(1, 10);
    m_rightLeader.ConfigPeakOutputReverse(-1, 10);

		/* set closed loop gains in slot0 */
    m_rightLeader.SelectProfileSlot(0, 0);
    m_rightLeader.Config_kF(0, 0.3, 10);
    m_rightLeader.Config_kP(0, 0.1, 10);
    m_rightLeader.Config_kI(0, 0.0, 10);
    m_rightLeader.Config_kD(0, 0.0, 10);

    m_rightLeader.ConfigMotionCruiseVelocity(7000, 10);
    m_rightLeader.ConfigMotionAcceleration(2000, 10);

    m_rightLeader.SetSelectedSensorPosition(0, 0, 10);
    
    m_rightLeader.SetSensorPhase(true);
    m_rightLeader.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
    m_rightLeader.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);


    m_leftLeader.SetSelectedSensorPosition(0, 0, 10);


    //m_rightLeader.ConfigFactoryDefault();
    int absolutePositionLeft = m_rightLeader.GetSensorCollection().GetPulseWidthPosition();
/* use the low level API to set the quad encoder signal */
		m_leftLeader.SetSelectedSensorPosition(absolutePositionLeft, kPIDLoopIdx,
				kTimeoutMs);
    m_leftLeader.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
		m_leftLeader.SetSensorPhase(true);

    m_leftLeader.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
    m_leftLeader.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

		/* set the peak and nominal outputs, 12V means full */
    m_leftLeader.ConfigNominalOutputForward(0, 10);
    m_leftLeader.ConfigNominalOutputReverse(0, 10);
    m_leftLeader.ConfigPeakOutputForward(1, 10);
    m_leftLeader.ConfigPeakOutputReverse(-1, 10);

		/* set closed loop gains in slot0 */
    m_leftLeader.SelectProfileSlot(0, 0);
    m_leftLeader.Config_kF(0, 0.3, 10);
    m_leftLeader.Config_kP(0, 0.1, 10);
    m_leftLeader.Config_kI(0, 0.0, 10);
    m_leftLeader.Config_kD(0, 0.0, 10);

    m_leftLeader.ConfigMotionCruiseVelocity(7000, 10);
    m_leftLeader.ConfigMotionAcceleration(2000, 10);

    m_leftLeader.SetSelectedSensorPosition(0, 0, 10);
    
    m_leftLeader.SetSensorPhase(true);
    m_leftLeader.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
    m_leftLeader.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
    // set PID coefficients


    //Grabber Rotate
    m_GrabberAnglePidController.SetP(kP);
    m_GrabberAnglePidController.SetI(kI);
    m_GrabberAnglePidController.SetD(kD);
    m_GrabberAnglePidController.SetIZone(kIz);
    m_GrabberAnglePidController.SetFF(kFF);
    m_GrabberAnglePidController.SetOutputRange(kMinOutput, kMaxOutput);

    m_GrabberAnglePidController.SetSmartMotionMaxVelocity(kMaxVel);
    m_GrabberAnglePidController.SetSmartMotionMinOutputVelocity(kMinVel);
    m_GrabberAnglePidController.SetSmartMotionMaxAccel(kMaxAcc);
    m_GrabberAnglePidController.SetSmartMotionAllowedClosedLoopError(kAllErr);

  }

  void AutonomousInit() override
  {
    //Set timer to zero and start counting
    m_timer.Reset();
    m_timer.Start();

    autoMode = frc::SmartDashboard::GetNumber("Auto mode", 0);

  }

  void AutonomousPeriodic() override
  {
    //Position of the right leader
    sensor = m_rightLeader.GetSensorCollection().GetQuadraturePosition();
    if (m_timer.Get() >= 0_s && m_timer.Get() <= 0.2_s){
      m_rightLeader.SetSelectedSensorPosition(0, 0, 10);
      m_leftLeader.SetSelectedSensorPosition(0, 0, 10);

      rahSetPoint = 0;
    } else if (m_timer.Get() >= 2_s && m_timer.Get() <= 2.05_s){
    m_rightLeader.SetSelectedSensorPosition(0, 0, 10);
    m_leftLeader.SetSelectedSensorPosition(0, 0, 10);

    rahSetPoint = 14920 * (161.375/(6*M_PI));
    ;
  }
  if (m_timer.Get() >= 0_s && m_timer.Get() <= 1_s){
    m_GrabberAnglePidController.SetReference(-4, rev::CANSparkMax::ControlType::kSmartMotion);
  } else if (m_timer.Get() >= 1_s && m_timer.Get() <= 1.5_s){
    m_GrabberIntake.Set(-1);
    m_GrabberAnglePidController.SetReference(-3.5, rev::CANSparkMax::ControlType::kSmartMotion);
  } else {
    m_GrabberIntake.Set(0);
    m_GrabberAnglePidController.SetReference(0, rev::CANSparkMax::ControlType::kSmartMotion);

  }
  
  double pitchAngleDegrees  = navx->GetPitch();

  if (fabs(pitchAngleDegrees) >= fabs(kOffBalanceThresholdDegreesActive) && m_timer.Get() >= 4_s && autoMode == 0){
    autoBalance = true;
  } 

  if (autoBalance == true) {
    double xAxisRate          = 0;
    //pitchAngleDegrees is the angle the robot is at
    double pitchAngleDegrees  = navx->GetPitch();

    //If the absolute value of the current angle of the robot is greater
    //the constant of what we consider is off balance, then change autoBalanceXMode to true;
    
    if ( !autoBalanceXMode &&
        (fabs(pitchAngleDegrees) >=
        fabs(kOffBalanceThresholdDegrees))) {
      autoBalanceXMode = true;
    }
    //If the absolute value of the current angle of the robot is less
    //the constant of what we consider is on balance, then change autoBalanceXMode to false;
    else if ( autoBalanceXMode &&
        (fabs(pitchAngleDegrees) <=
        fabs(kOnBalanceThresholdDegrees))) {
      autoBalanceXMode = false;
    }

    // If autoBalanceXMode is true, then calculate the speed of the robot (sin(Current angle * pi/180) * -1)
    // yes, we're actually using trig in real life for the first time ever.
    if ( autoBalanceXMode ) {
      double pitchAngleRadians = pitchAngleDegrees * (M_PI / 180.0);
      xAxisRate = sin(pitchAngleRadians);
    }

    // Set the drive speed of the robot to 60% of the speed we just calculated. This 60% may need to be changed again
    // once we test the autobalance code on the actual balance board.
    m_rightLeader.Set(ControlMode::PercentOutput, (0.6)*xAxisRate);
    m_leftLeader.Set(ControlMode::PercentOutput, -(0.6)*xAxisRate);


  } else {
      m_rightLeader.Set(ControlMode::MotionMagic, rahSetPoint);
      m_leftLeader.Set(ControlMode::MotionMagic, -rahSetPoint);

  } 

  if (autoMode == 2){
    m_rightLeader.Set(ControlMode::PercentOutput, 0);
    m_leftLeader.Set(ControlMode::PercentOutput, 0);
    fmt::print("STOP={}\n", 1==1);


  }

    m_rightFollower.Follow(m_rightLeader);
    m_leftFollower.Follow(m_leftLeader);


  }

  void TeleopInit() override 
  {

    m_leftLeader.ConfigFactoryDefault();
    m_rightLeader.ConfigFactoryDefault();
    m_leftFollower.ConfigFactoryDefault();
    m_rightFollower.ConfigFactoryDefault();

    m_leftFollower.SetInverted(InvertType::FollowMaster);

    SetPointGrabberAngle = 0;
    //m_rightFollower.Follow(m_rightLeader);
    //m_leftFollower.Follow(m_leftLeader);
  }

  void TeleopPeriodic() override
  {


    m_leftFollower.Follow(m_leftLeader);
    m_rightFollower.Follow(m_rightLeader);
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
    //double turn = m_turnRateLimiter.Calculate(m_stick.GetTwist());
    double turn = (0.5) * ((m_stick.GetTwist())*(fabs(m_stick.GetTwist())));
    double turnLimit = turn * (-((fabs(speed)) / 2) + 1);


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
      m_leftLeader.Set(ControlMode::PercentOutput, -(0.3)*speed, DemandType::DemandType_ArbitraryFeedForward, (0.5)*turnLimit);
      m_rightLeader.Set(ControlMode::PercentOutput, (0.3)*speed, DemandType::DemandType_ArbitraryFeedForward,(0.5)*turnLimit);
      m_leftFollower.Follow(m_leftLeader);
      m_rightFollower.Follow(m_rightLeader);
    } else {
      m_leftLeader.Set(ControlMode::PercentOutput, -speed, DemandType::DemandType_ArbitraryFeedForward, turnLimit);
      m_rightLeader.Set(ControlMode::PercentOutput, speed, DemandType::DemandType_ArbitraryFeedForward, turnLimit);
      m_leftFollower.Follow(m_leftLeader);
      m_rightFollower.Follow(m_rightLeader);
    }

//_________________________________________________________________________________________________________
//Grabber subsystem

    m_runGrabberIntakeIn = m_xbox.GetLeftTriggerAxis() > 0.1;
    m_runGrabberIntakeOut = m_xbox.GetRightTriggerAxis() > 0.1;


    if (m_runGrabberIntakeIn && m_runGrabberIntakeOut)
    {
      m_GrabberIntake.Set(0);
    } else if (m_runGrabberIntakeIn)
    {
      m_GrabberIntake.Set(0.45);

    } else if (m_runGrabberIntakeOut)
    {
      m_GrabberIntake.Set(-1);
    } else {
      m_GrabberIntake.Set(0);
    }



      if (m_xbox.GetYButtonPressed()){

        SetPointGrabberAngle = -3.345;
      
      }
      if (m_xbox.GetBButtonPressed()){

        SetPointGrabberAngle = 0;
      
      }
      if (m_xbox.GetAButtonPressed()){

        SetPointGrabberAngle = 3.345;
      
      }
      if (m_xbox.GetXButtonPressed()){

        SetPointGrabberAngle = 1.673;
      
      }

      if (m_GrabberAngleEncoder.GetPosition() > SetPointGrabberAngle + 0.1 || m_GrabberAngleEncoder.GetPosition() < SetPointGrabberAngle - 0.1){
        m_GrabberAnglePidController.SetReference(SetPointGrabberAngle, rev::CANSparkMax::ControlType::kSmartMotion);
      }
      if (m_GrabberAngleEncoder.GetPosition() > 5 || m_GrabberAngleEncoder.GetPosition() < -5){
        m_GrabberAngle.Set(0);
        fmt::print("motordisabled={}\n", 1==1);

      }

    //fmt::print("yawAngleDegrees={}\n", navx->GetYaw());
    /*
    if (m_stick.GetRawButtonPressed(4)){
      navx->ZeroYaw();
      yawAngleDegrees  = navx->GetYaw();
      turnAround = !turnAround;
      
    }
    */

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

  double yawAngleDegrees  = 0;

  double SetPointGrabberAngle;


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

  //exactly what you think, its a timer
  frc::Timer m_timer;

  //Set up gyro
  AHRS *navx;

  //Constants for autobalance, 10 degrees for off balance and 5 for on balance. 
  //I want to mess around with these constants at some point. 
  constexpr static const double kOffBalanceThresholdDegrees = 5.1f;
  constexpr static const double kOffBalanceThresholdDegreesActive = 7.0f;

  constexpr static const double kOnBalanceThresholdDegrees  = 5.0f;
  bool autoBalanceXMode = false;
  bool autoBalanceYMode = false;

  bool turnAround;
  

  //Grabber motors
  //TalonSRX m_GrabberIntake = {kGrabberIntakeID};
  //TalonSRX m_GrabberAngle = {kGrabberAngleID};
  rev::CANSparkMax m_GrabberIntake{kGrabberIntakeID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_GrabberAngle{kGrabberAngleID, rev::CANSparkMax::MotorType::kBrushless};

  rev::SparkMaxRelativeEncoder m_GrabberAngleEncoder = m_GrabberAngle.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
  rev::SparkMaxPIDController m_GrabberAnglePidController = m_GrabberAngle.GetPIDController();

  //Grabber constants
  bool m_runGrabberIntakeIn = false;
  bool m_runGrabberIntakeOut = false;
  int grabberCount = 0;
  bool grabberAngleOn = false;


  //Grabber subsystem (currently broken yay :D)
  //GrabberSubsystem m_grabber{m_GrabberIntake, m_GrabberAngle, m_xbox};


  // default PID coefficients
  double kP = 5e-5, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000156, kMaxOutput = 1, kMinOutput = -1;

  double kP2 = 5e-5, kI2 = 1e-6, kD2 = 0, kIz2 = 0, kFF2 = 0.000156, kMaxOutput2 = 1, kMinOutput2 = -1;

  // default smart motion coefficients
  double kMaxVel = 2000, kMinVel = 0, kMaxAcc = 1500, kAllErr = 0;

  int autoSteps = 0;
  int sensor = 0;

  // motor max RPM
  const double MaxRPM = 5700;  

  //Arm constants
  bool armRotateOn = false;
  int armCount = 0;
  double rahSetPoint;
  int rahCount = 0;
	int _loops = 0;
	bool _lastButton1 = false;
	/** save the target position to servo to */
	double targetPositionRotations;

  bool autoBalance = false;
  int autoMode;

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