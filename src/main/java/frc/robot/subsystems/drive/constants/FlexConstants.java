// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class FlexConstants {
  public static final double maxSpeedMetersPerSec = Units.inchesToMeters(19.8 * 12); // 6.035
  public static final double maxAcceleration = 2.0;
  public static final double allowedError = 0.05;
  public static final double trackWidth = Units.inchesToMeters(28.0);
  public static final double wheelBase = Units.inchesToMeters(28.0);

  // Zeroed rotation values for each module (not necessary, calibration done with client)
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0); // 0.2106459
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(0); // 0.1326632
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(0); // 0.8751022
  public static final Rotation2d backRightZeroRotation = new Rotation2d(0); // 0.8419363

  // Drive motor configuration
  public static final boolean driveInverted = true;
  public static final int driveMotorCurrentLimit = 60;
  public static final double wheelRadiusMeters = Units.inchesToMeters(2);
  public static final double driveMotorReduction =
      5.14; // ThriftyBot with 14 pinion teeth and 16 spur teeth
  public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

  // Drive PID configuration for the robot
  public static final double driveKp = 0.005;
  public static final double driveKd = 0.02;
  public static final double driveKs = 0.14819;
  public static final double driveKv = 0.0898;

  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 40;
  public static final double turnMotorReduction = 25.0;
  public static final DCMotor turnGearbox = DCMotor.getNeoVortex(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = false;

  public static final double absoluteTurnEncoderPositionFactor = 2 * Math.PI;
  public static final double absoluteTurnEncoderVelocityFactor =
      absoluteTurnEncoderPositionFactor / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 1.6;
  public static final double turnKd = 1.8;

  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;

  // PathPlanner configuration
  public static final double robotMassKg = 55.34;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.0;
}
