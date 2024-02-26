// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

//core
#include <fstream>
#include <string>
#include <sstream>
#include <units/dimensionless.h>
#include <fmt/core.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>

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
#include <frc/DigitalInput.h>

//motors
#include <rev/CANSparkMax.h>
#include <ctre/phoenix6/TalonFX.hpp>

//camera
#include <cameraserver/CameraServer.h>

//gryo
#include <AHRS.h>
#include <frc/SerialPort.h>

//local
#include <Constants.h>

namespace phx = ctre::phoenix6;

class Robot : public frc::TimedRobot {
  private:
    static constexpr char const *CAN{"rio"};

    /**
     * CONFIGURE DRIVE MOTORS
     * FIXME: voltageControl is UNTESTED
    */

    /* Voltage control */
    //phx::controls::VoltageOut voltageControl{0_V};

    /* Drive configuration */
    phx::hardware::TalonFX leftLeader{can::leftLeader, CAN};
    phx::hardware::TalonFX leftFollower{can::leftFollower, CAN};
    phx::hardware::TalonFX rightLeader{can::rightLeader, CAN};
    phx::hardware::TalonFX rightFollower{can::rightFollower, CAN};
    
    phx::controls::DutyCycleOut leftOut{0}; // Initialize output to 0%
    phx::controls::DutyCycleOut rightOut{0}; // Initialize output to 0%

    /**
     * CONFIGURE ARM/WRIST MOTORS
    */

    phx::hardware::TalonFX arm{can::arm, CAN};
    phx::hardware::TalonFX wrist{can::wrist, CAN};

    phx::controls::MotionMagicExpoDutyCycle mmArm{arm::home};
    phx::controls::MotionMagicExpoDutyCycle mmWrist{wrist::home};

    enum class Mech {Home, Intake, Delivery, AmpScore, Release, Climb};
    Mech mechMode = Mech::Home;
    bool armMoving = false;
    bool wristMoving = false;

    // Slow drive starts as false, is enabled by pressing button 3
    bool slowDrive = false;

    /* joystick USB port connection (assigned in driver station)
      Make sure these are correctly assigned in the driver station, if they aren't the robot can't read any inputs */

    frc::Joystick joystick{0};
    frc::XboxController xbox{1};
    
    // intake sensor
    bool beamBreak = false;
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
    Robot () {
      /* set up gyro */
      navx = new AHRS(frc::SPI::Port::kMXP);

      /* start timer */
      m_timer.Start();
    }

    void RobotInit() override;
    //void RobotPeriodic() override;

    //void AutonomousInit() override;
    //void AutonomousPeriodic() override;

    //void TeleopInit() override;
    void TeleopPeriodic() override;

    //void DisabledInit() override;
    void DisabledPeriodic() override;

    //void TestInit() override;
    //void TestPeriodic() override;

    //void SimulationInit() override;
    //void SimulationPeriodic() override;
};
