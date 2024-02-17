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

//camera
#include <cameraserver/CameraServer.h>

//gryo
#include "AHRS.h"
#include <frc/SerialPort.h>

//local
#include "Constants.h"

using namespace ctre::phoenix6;

class Robot : public frc::TimedRobot {
  private:

    // Slow drive starts as false, is enabled by pressing button 3
    bool slowDrive = false;
  
    // Talon drive motors
    /* This can be a CANivore name, CANivore serial number,
      SocketCAN interface, or "*" to select any CANivore. */
    static constexpr char const *CAN = "*";

    /* devices */
    hardware::TalonFX leftLeader{canLeftLeader, CAN};
    hardware::TalonFX leftFollower{canLeftFollower, CAN};
    hardware::TalonFX rightLeader{canRightLeader, CAN};
    hardware::TalonFX rightFollower{canRightFollower, CAN};

    /* control requests */
    controls::DutyCycleOut leftOut{0};
    controls::DutyCycleOut rightOut{0};


    /* joystick USB port connection (assigned in driver station)
      Make sure these are correctly assigned in the driver station, if they aren't the robot can't read any inputs */
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
    Robot() {
      // Set up gyro
      // FIXME - shouldn't this be in RobotInit() ?
      navx = new AHRS(frc::SPI::Port::kMXP);
      m_timer.Start();
    }
    */

    /*
    * Runs once at code initialization.
    */
    void RobotInit() override {

      frc::SmartDashboard::PutNumber("Auto mode", 0);

      frc::CameraServer::StartAutomaticCapture();
      frc::CameraServer::StartAutomaticCapture();

      configs::TalonFXConfiguration fx_cfg{};

      // Set up gyro
      // FIXME - shouldn't this be in RobotInit() ?
      navx = new AHRS(frc::SPI::Port::kMXP);
      m_timer.Start();

      /* the left motor is CCW+ */
      fx_cfg.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
      leftLeader.GetConfigurator().Apply(fx_cfg);

      /* the right motor is CW+ */
      fx_cfg.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
      rightLeader.GetConfigurator().Apply(fx_cfg);

      /* set follower motors to follow leaders; do NOT oppose the leaders' inverts */
      leftFollower.SetControl(controls::Follower{leftLeader.GetDeviceID(), false});
      rightFollower.SetControl(controls::Follower{rightLeader.GetDeviceID(), false});
    }

    /**
     * FIXME
    */
    //void RobotPeriodic() {}

    /**
     * Runs when transitioning from enabled to disabled,
     * including after robot startup.
     */
    //void DisabledInit() {}

    /**
     * Runs periodically while disabled.
     */
    
    void DisabledPeriodic() {
      leftLeader.SetControl(controls::NeutralOut{});
      rightLeader.SetControl(controls::NeutralOut{});
    }
    

    //void AutonomousInit() override;

    //void AutonomousPeriodic() override;

    //void TeleopInit() override;

    void TeleopPeriodic() override {
      /**
       * SLOW MODE
       * Button three on the joystick toggles "slow drive" mode.
       * In this mode, the robot's drive and turn speed are limited to the slowFactor value.
      */
      if (joy.GetRawButtonPressed(3)){
        slowDrive = !slowDrive;
      }
      const double slowFactor = 0.3;

      /**
       * JITTER CORRECTION
       * throw out any inputs less than deadband value
      */
      const double deadband = 0.05;
      double speed = (fabs(speed) > deadband) ? joy.GetY() : 0.0;
      
      /* taking half of the signed square of the twist...
          must be a really slow turn by default */
      double turn = (0.5) * ((joy.GetTwist())*(fabs(joy.GetTwist())));
      double turnLimit = turn * (-((fabs(speed)) / 2) + 1);
      
      /**
       * set drive outputs
       * FIXME: not sure how to apply turnLimit here
      */
      leftOut.Output = slowDrive ? slowFactor*(speed + turn) : speed + turn; // 
      rightOut.Output = slowDrive ? slowFactor*(speed + turn) : speed + turn; // not sure how to apply turnLimit here

      leftLeader.SetControl(leftOut);
      rightLeader.SetControl(rightOut);

    }

    //void TestInit() override;

    //void TestPeriodic() override;
    
    //void SimulationInit() override;
    
    //void SimulationPeriodic() override;

};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif