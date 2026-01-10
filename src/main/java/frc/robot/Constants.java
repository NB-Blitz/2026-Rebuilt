// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final Transform2d[] leftReef =
      new Transform2d[] {
        new Transform2d(0.6, -0.185, Rotation2d.fromDegrees(180.0)),
        new Transform2d(0.5, -0.21, Rotation2d.fromDegrees(180.0)),
        new Transform2d(0.4, -0.185, Rotation2d.fromDegrees(180.0)),
        new Transform2d(0.45, -0.19, Rotation2d.fromDegrees(180.0))
      };
  public static final Transform2d[] rightReef =
      new Transform2d[] {
        new Transform2d(0.6, 0.165, Rotation2d.fromDegrees(180.0)),
        new Transform2d(0.5, 0.155, Rotation2d.fromDegrees(180.0)),
        new Transform2d(0.4, 0.165, Rotation2d.fromDegrees(180.0)),
        new Transform2d(0.45, 0.16, Rotation2d.fromDegrees(180.0))
      };

  public static final PIDController xController = new PIDController(4, 0, 0);
  public static final PIDController yController = new PIDController(4, 0, 0);
  public static final PIDController thetaController = new PIDController(7, 0, 0);

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
