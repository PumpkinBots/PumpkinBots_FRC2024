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
 * drv : Drive namespace
 * arm : Functional namespace
 * wrist : Functional namespace
*/

//namespace OperatorConstants {

//constexpr int driverControllerPort = 0;

//}  // namespace OperatorConstants

#include <units/angle.h>

using rot = units::angle::turn_t;

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

namespace dio {
	static constexpr int noteSensor = 0;
}

namespace intake {
	static constexpr double intakeOut = 0.2; // FIXME - this should be maximized for shooting
}

/**
 * at full speed, a Kraken is 6K rpm <- only used for estimating time to reach position
*/
namespace arm {
	static constexpr double gearOut = (5*4*3) * (52/15); // gearIn is assumed 1, planetary gearbox is 3:1/4:1/5:1 (60:1),  chain drive ratio is 52:15
	static constexpr rot home = (rot) gearOut * (0/360); // 0°
	static constexpr rot intake = (rot) gearOut * (0/360); // 0°
	static constexpr rot amp = (rot) gearOut * (100/360); // 100°
	static constexpr rot climb = (rot) gearOut * (90/360); // 90°
}

namespace wrist {
	static constexpr double gearOut = (5*5*5) * (15/10); // gearIn is assumed 1, planetary gearbox is 3x5:1 (125:1), chain drive ratio is 15:10
	static constexpr rot home = (rot) gearOut * (0/360); // 0°
	static constexpr rot intake = (rot) gearOut * (0/360); // 0°
	static constexpr rot amp = (rot) gearOut * (35/360); // 35°
	static constexpr rot climb = (rot) gearOut * (150/360); // 150°
}