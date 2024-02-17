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

namespace OperatorConstants {

constexpr int driverControllerPort = 0;

}  // namespace OperatorConstants
static constexpr int canLeftLeader = 0;
static constexpr int canLeftFollower = 1;
static constexpr int canRightLeader = 2;
static constexpr int canRightFollower = 3;

/*
static constexpr int kGrabberArm = 8;
static constexpr int kGrabberWrist = 9;
static constexpr int kGrabberIntakeLeader = 10;
static constexpr int kGrabberIntakeFollower = 11;
*/
//double grabberStart = 0;

//double kSlotIdx = 0;

	/* Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops.
	 * For now we just want the primary one.
	 */
//double kPIDLoopIdx = 0;

	/*
	 * set to zero to skip waiting for confirmation, set to nonzero to wait
	 * and report to DS if action fails.
	 */
//double kTimeoutMs = 30;