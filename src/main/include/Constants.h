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
	static constexpr double intakeOut = 0.2;
}

/**
 * at full speed, a Kraken is 6K rpm
 * eg gearOut = 60; // 3:1 + 4:1 + 5:1
 * the wrist is 1:1 
 * speed of the output shaft on the gearbox = speed of the wrist rotation
 * the rollers are 30:24 or 5:4 // this shouldn't matter much
 * and the arm is 52:15
 * so whatever gearbox you guys use for the arm, for every time that rotates once, the arm will rotate about 1/5 of a rotation
*/
namespace arm {
	static constexpr double armGearOut = 60.0 * 52/15; // gearIn is assumed 1, planetary gearbox is 3:1/4:1/5:1 (60:1),  chain drive ratio is 52:15
	static constexpr rot home = (rot) armGearOut * (0/360); // 0 degrees
	static constexpr rot intake = (rot) armGearOut * (0/360); // 0 degrees
	static constexpr rot amp = (rot) armGearOut * (100/360); // 100 degrees
	static constexpr rot climb = (rot) armGearOut * (90/360); // 90 degrees
}

//DEBUG_MSG("Arm position: " << 360 * arm.GetPosition().GetValueAsDouble() / arm::gearOut << " degrees");
// 

namespace wrist {
	static constexpr double wristGearOut = 125.0; // gearIn is assumed 1, planetary gearbox is 3*5:1 (125:1), belt drive ratio is 1:1
	static constexpr rot home = (rot) wristGearOut * (150/360); // 150 degrees
	static constexpr rot intake = (rot) wristGearOut * (0/360); // 0 degrees
	static constexpr rot amp = (rot) wristGearOut * (35/360); // 35 degrees
	static constexpr rot climb = (rot) wristGearOut * (150/360); // 150 degrees
}

/**
 * XBOX CONTROLLER CONFIGURATION
 * A: 1
 * B: 2
 * X: 3
 * Y: 4
 * L1: 5
 * R1: 6
*/