// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.superstructure;

public class SuperstructureConstants {
  public static final int feederCanId = 10;
  public static final double feederMotorReduction = 8.0;
  public static final int feederCurrentLimit = 60;

  public static final int launcherCanId = 11;
  public static final int launcherFollowerCanId = 107; // FIXME
  public static final double launcherMotorReduction = 1.0;
  public static final int launcherCurrentLimit = 60;

  public static final int intakeMotorCanId = 9; // FIXME
  public static final double intakeMotorReduction = 2.0; // FIXME
  public static final int intakeCurrentLimit = 60; // FIXME

  public static final int sweeperCanId = 12;
  public static final int sweeperCurrentLimit = 20;

  public static final double intakingFeederSpeed = -726.0; // -1.0;
  public static final double intakingIntakeSpeed = 1500.0; // 10.0 / 12.0;
  public static final double launchingFeederSpeed = 725.0; // 9.0 / 12.0;
  public static final double launchingLauncherSpeed = 3000.0; // 10.6 / 18.0;
  public static final double launchingIntakeSpeed = 1500.0; // 10.0 / 12.0;
  public static final double spinUpFeederSpeed = -725.0; // -6.0 / 12.0;
  public static final double spinUpSeconds = 1.0;
  public static final double sweeperSpeed = 0.00; // -0.5

  public static final double feederKp = 0.0001;
  public static final double feederKd = 0.0;
  public static final double feederKv = 0.014; // 0.00122;
  public static final double launcherKp = 0.00002;
  public static final double launcherKd = 0.0;
  public static final double launcherKv = 0.0018; // 0.0001495
  public static final double intakeKp = 0.000075;
  public static final double intakeKd = 0.0;
  public static final double intakeKv = 0.0038; // 0.00035;
}
