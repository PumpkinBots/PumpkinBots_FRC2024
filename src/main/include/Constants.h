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

constexpr int kDriverControllerPort = 0;



}  // namespace OperatorConstants

static constexpr int kDriveLeftLeader = 1;
static constexpr int kDriveLeftFollower = 4;
static constexpr int kDriveRightLeader = 2;
static constexpr int kDriveRightFollower = 9;

static constexpr int kGrabberIntakeID = 3;
static constexpr int kGrabberAngleID = 5;

static constexpr int kArmRotateID = 6;
static constexpr int kArmRetractID = 7;