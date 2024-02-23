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
	static constexpr int leftLeader = 0;
	static constexpr int leftFollower = 1;
	static constexpr int rightLeader = 2;
	static constexpr int rightFollower = 3;

	/* reserve addresses 4:7 for swerve drive */

	static constexpr int arm = 8;
	static constexpr int wrist = 9;
	static constexpr int intakeLeader = 10;
	static constexpr int intakeFollower = 11;
}

/**
 * FIXME: all of the definitions below DO NOT account for gear ratios - they will need to be adjusted
 * eg gearOut = 60;
 * the wrist is 1:1 
 * speed of the output shaft on the gearbox = speed of the wrist rotation
 * the rollers are 30:24 or 5:4 // this shouldn't matter much
 * and the arm is 52:10 or 26:5 
 * so whatever gearbox you guys use for the arm, for every time that rotates once, the arm will rotate about 1/5 of a rotation
*/
namespace arm {
	static constexpr double gearOut = 1.0; // gearIn is assumed 1 -- this needs to encompass the 26:5 belt drive ratio
	static constexpr rot home = (rot) gearOut * 0.0; // `static constexpr rot home = gearOut*0.0_tr;` doesn't work
    static constexpr rot intake = (rot) gearOut * 0.0;
	static constexpr rot amp = (rot) gearOut * 100/360;
	static constexpr rot climb = (rot) gearOut * 90/360;
}

namespace wrist {
	static constexpr double gearOut = 1.0; // gearIn is assumed 1 -- belt drive ratio is 1:1
	static constexpr rot home = (rot) gearOut * 150/360;
    static constexpr rot intake = (rot) gearOut * 0.0;
	static constexpr rot amp = (rot) gearOut * 35/360;
	static constexpr rot climb = (rot) gearOut * 150/360;
}