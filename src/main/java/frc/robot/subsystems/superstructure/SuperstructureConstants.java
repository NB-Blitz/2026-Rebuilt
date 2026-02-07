// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.superstructure;

public class SuperstructureConstants {
  public static final int feederCanId = 10;
  public static final double feederMotorReduction = 1.0;
  public static final int feederCurrentLimit = 60;

  public static final int launcherCanId = 11;
  public static final int launcherFollowerCanId = 107; // FIXME
  public static final double launcherMotorReduction = 1.0;
  public static final int launcherCurrentLimit = 60;

  public static final int intakeMotorCanId = 9; // FIXME
  public static final double intakeMotorReduction = 1.0; // FIXME
  public static final int intakeCurrentLimit = 60; // FIXME

  public static final double intakingFeederSpeed = -1.0;
  public static final double intakingIntakeSpeed = 10.0 / 12.0;
  public static final double launchingFeederSpeed = 9.0 / 12.0;
  public static final double launchingLauncherSpeed = 10.6 / 18.0;
  public static final double spinUpFeederSpeed = -6.0 / 12.0;
  public static final double spinUpSeconds = 1.0;
}
