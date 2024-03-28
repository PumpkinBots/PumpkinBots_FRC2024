// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

/**
 * ctl : Control namespace (eg joystick, xbox)
 * can : CANBUS namespace (addresses)
 * drv : Drive namespace UNUSED
 * arm : Mechanism namespace
 * wrist : Mechanism namespace
 * intake : 
*/

#include <units/angle.h>

using rot = units::angle::turn_t;
using deg = units::angle::degree_t;

namespace can {
	static constexpr int leftDrive = 0;
	static constexpr int leftFollower = 1;
	static constexpr int rightDrive = 2;
	static constexpr int rightFollower = 3;

	/* reserve addresses 4:7 for swerve drive */

	static constexpr int arm = 8;
	static constexpr int armFollower = 9;
	static constexpr int wrist = 10;
	static constexpr int intake = 11;
	static constexpr int intakeFollower = 12;
}

/**
 * joystick and xbox USB port connection (assigned in driver station)
 * Make sure these are correctly assigned in the driver station, if they aren't the robot can't read any inputs
*/
namespace ctl {
	static constexpr int joystick = 0;
	static constexpr int xbox = 1;
}

namespace dio {
	static constexpr int noteSensor = 0;
}

namespace power {
	static constexpr double intakePlace = 0.2;
	static constexpr double intakeShot = 1.0;
	static constexpr double armClimb = 0.2;
	static constexpr double armPeak = 0.75; //0.75 FIXME
	static constexpr double wristPeak = 0.75;
}

/**
 * ARM and WRIST gearing and setpoint configuration
 * phoenix6 API uses rotational units (units::angle::turn_t) natively
 * at full speed, a Kraken is 6K rpm <- only used for estimating time to reach position
*/
namespace arm {
	static constexpr double gearOut = 3*4*5 * 52/15;// gearIn is assumed 1, planetary gearbox is 3:1/4:1/5:1 (60:1),  chain drive ratio is 52:15
	static constexpr rot home{gearOut * deg{0}}; // 0°
	static constexpr rot intake{gearOut * deg{0}}; // 0°
	static constexpr rot amp{gearOut * deg{100}}; // 100°
	static constexpr rot climb{gearOut * deg{90}}; // 90°
	static constexpr rot climbDown{gearOut * deg{25}}; // 25°
}

namespace wrist {
	static constexpr double gearOut = 5*5*5 * 15/10; // gearIn is assumed 1, planetary gearbox is 3x5:1 (125:1), chain drive ratio is 15:10
	static constexpr rot home{gearOut * deg{150}}; // 150°
	static constexpr rot intake{gearOut * deg{0}}; // 0°
	static constexpr rot amp{gearOut * deg{25}}; // 25°
	static constexpr rot climb{gearOut * deg{150}}; // 150°
}