// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//core
#include <fstream>
#include <string>
#include <sstream>
#include <units/dimensionless.h>
#include <fmt/core.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"

//frc
#include <frc/Filesystem.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/filter/SlewRateLimiter.h>

//motors
#include "rev/CANSparkMax.h"
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/unmanaged/Unmanaged.hpp" // for FeedEnable

//camera
#include <cameraserver/CameraServer.h>

//gryo
#include "AHRS.h"
#include <frc/SerialPort.h>

//local
#include "Constants.h"

//using namespace ctre::phoenix6;
namespace phx = ctre::phoenix6;

class Robot : public frc::TimedRobot {
  private:

    // Slow drive starts as false, is enabled by pressing button 3
    bool slowDrive = false;
  
    // Talon drive motors
    /* This can be a CANivore name, CANivore serial number,
      SocketCAN interface, or "*" to select any CANivore. */
    static constexpr char const *CAN = "*";

    /* devices */
    phx::hardware::TalonFX leftLeader{can::leftLeader, CAN};
    phx::hardware::TalonFX leftFollower{can::leftFollower, CAN};
    phx::hardware::TalonFX rightLeader{can::rightLeader, CAN};
    phx::hardware::TalonFX rightFollower{can::rightFollower, CAN};

    /* control requests */
    phx::controls::DutyCycleOut leftOut{0};
    phx::controls::DutyCycleOut rightOut{0};


    /* joystick USB port connection (assigned in driver station)
      Make sure these are correctly assigned in the driver station, if they aren't the robot can't read any inputs */
      // FIXME - convert to ctl namespace (in header?)
    frc::Joystick joy{0};
    frc::XboxController xbox{1};
    
    //Set up slew rate limiter

    //exactly what you think, its a timer
    frc::Timer m_timer;

    //Set up gyro
    AHRS *navx;

    // Allow the robot to access the data from the camera. 
    std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
    double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
    double targetArea = table->GetNumber("ta",0.0);
    double targetSkew = table->GetNumber("ts",0.0);

  public:

    /*
    * Runs once at code initialization.
    */
    void RobotInit() override {

      frc::SmartDashboard::PutNumber("Auto mode", 0);

      frc::CameraServer::StartAutomaticCapture();
      frc::CameraServer::StartAutomaticCapture();

      /* set up gyro */
      navx = new AHRS(frc::SPI::Port::kMXP);
      m_timer.Start();

      /**
       * DRIVE MOTOR CONFIGURATION
      */

      ctre::phoenix::unmanaged::FeedEnable(100);

      phx::configs::TalonFXConfiguration fx_cfg{};

      /* the left motor is CCW+ */
      fx_cfg.MotorOutput.Inverted = phx::signals::InvertedValue::CounterClockwise_Positive;
      leftLeader.GetConfigurator().Apply(fx_cfg);

      /* the right motor is CW+ */
      fx_cfg.MotorOutput.Inverted = phx::signals::InvertedValue::Clockwise_Positive;
      rightLeader.GetConfigurator().Apply(fx_cfg);

      /* set follower motors to follow leaders; do NOT oppose the leaders' inverts */
      leftFollower.SetControl(phx::controls::Follower{leftLeader.GetDeviceID(), false});
      rightFollower.SetControl(phx::controls::Follower{rightLeader.GetDeviceID(), false});
    }

    void DisabledPeriodic() {
      leftLeader.SetControl(phx::controls::NeutralOut{});
      rightLeader.SetControl(phx::controls::NeutralOut{});
    }

    void TeleopPeriodic() override {
      /**
       * SLOW DRIVE
       * Button three on the joystick toggles "slow drive" mode.
       * In this mode, the robot's drive and turn speed are limited to the slowFactor value.
      */
      if (joy.GetRawButtonPressed(3)){
        slowDrive = !slowDrive;
      }
      const double slowFactor = 0.3;

      /**
       * SPEED
       * jitter correction: throw out any inputs less than the deadband value
      */
      const double deadband = 0.05;
      double speed = (fabs(speed) > deadband) ? joy.GetY() : 0.0;
      
      /**
       * TURN
       * taking half of the signed square of the twist... must be a really slow turn by default
      */
      double turn = (0.5) * ((joy.GetTwist())*(fabs(joy.GetTwist())));
      double turnLimit = turn * (-((fabs(speed)) / 2) + 1);
      
      /**
       * DRIVE OUTPUT (speed + turn)
       * FIXME: not sure how to apply turnLimit here
      */
      leftOut.Output = slowDrive ? slowFactor*(speed + turn) : speed + turn; // 
      rightOut.Output = slowDrive ? slowFactor*(speed + turn) : speed + turn; // not sure how to apply turnLimit here

      leftLeader.SetControl(leftOut);
      rightLeader.SetControl(rightOut);

    }

};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif