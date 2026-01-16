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

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class DriveConstants {
  public static final boolean compRobot = true;

  public static final double maxSpeedMetersPerSec =
      compRobot ? FlexConstants.maxSpeedMetersPerSec : SDSConstants.maxSpeedMetersPerSec;
  public static final double maxAcceleration =
      compRobot ? FlexConstants.maxAcceleration : SDSConstants.maxAcceleration;
  public static final double allowedError =
      compRobot ? FlexConstants.allowedError : SDSConstants.allowedError;
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth =
      compRobot ? FlexConstants.trackWidth : SDSConstants.trackWidth;
  public static final double wheelBase =
      compRobot ? FlexConstants.wheelBase : SDSConstants.wheelBase;
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  // Zeroed rotation values for each module
  public static final Rotation2d frontLeftZeroRotation =
      compRobot ? FlexConstants.frontLeftZeroRotation : SDSConstants.frontLeftZeroRotation;
  public static final Rotation2d frontRightZeroRotation =
      compRobot ? FlexConstants.frontRightZeroRotation : SDSConstants.frontRightZeroRotation;
  public static final Rotation2d backLeftZeroRotation =
      compRobot ? FlexConstants.backLeftZeroRotation : SDSConstants.backLeftZeroRotation;
  public static final Rotation2d backRightZeroRotation =
      compRobot ? FlexConstants.backRightZeroRotation : SDSConstants.backRightZeroRotation;

  // Device CAN IDs
  public static final int frontLeftDriveCanId = 1;
  public static final int backLeftDriveCanId = 3;
  public static final int frontRightDriveCanId = 5;
  public static final int backRightDriveCanId = 7;

  public static final int frontLeftTurnCanId = 2;
  public static final int backLeftTurnCanId = 4;
  public static final int frontRightTurnCanId = 6;
  public static final int backRightTurnCanId = 8;

  public static final int flTurningEncoderCANId =
      compRobot ? -1 : SDSConstants.flTurningEncoderCANId;
  public static final int frTurningEncoderCANId =
      compRobot ? -1 : SDSConstants.frTurningEncoderCANId;
  public static final int blTurningEncoderCANId =
      compRobot ? -1 : SDSConstants.blTurningEncoderCANId;
  public static final int brTurningEncoderCANId =
      compRobot ? -1 : SDSConstants.brTurningEncoderCANId;

  // Drive motor configuration
  public static final boolean driveInverted =
      compRobot ? FlexConstants.driveInverted : SDSConstants.driveInverted;
  public static final int driveMotorCurrentLimit =
      compRobot ? FlexConstants.driveMotorCurrentLimit : SDSConstants.driveMotorCurrentLimit;
  public static final double wheelRadiusMeters =
      compRobot ? FlexConstants.wheelRadiusMeters : SDSConstants.wheelRadiusMeters;
  public static final double driveMotorReduction =
      compRobot ? FlexConstants.driveMotorReduction : SDSConstants.driveMotorReduction;
  public static final DCMotor driveGearbox =
      compRobot ? FlexConstants.driveGearbox : SDSConstants.driveGearbox;

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Radians
  public static final double driveEncoderVelocityFactor =
      driveEncoderPositionFactor / 60.0; // Rotor Meters -> Radians/Sec

  // Drive PID configuration
  public static final double driveKp = compRobot ? FlexConstants.driveKp : SDSConstants.driveKp;
  public static final double driveKd = compRobot ? FlexConstants.driveKd : SDSConstants.driveKd;
  public static final double driveKs = compRobot ? FlexConstants.driveKs : SDSConstants.driveKs;
  public static final double driveKv = compRobot ? FlexConstants.driveKv : SDSConstants.driveKv;

  public static final double driveSimP =
      compRobot ? FlexConstants.driveSimP : SDSConstants.driveSimP;
  public static final double driveSimD =
      compRobot ? FlexConstants.driveSimD : SDSConstants.driveSimD;
  public static final double driveSimKs =
      compRobot ? FlexConstants.driveSimKs : SDSConstants.driveSimKs;
  public static final double driveSimKv =
      compRobot ? FlexConstants.driveSimKv : SDSConstants.driveSimKv;

  // Turn motor configuration
  public static final boolean turnInverted =
      compRobot ? FlexConstants.turnInverted : SDSConstants.turnInverted;
  public static final int turnMotorCurrentLimit =
      compRobot ? FlexConstants.turnMotorCurrentLimit : SDSConstants.turnMotorCurrentLimit;
  public static final double turnMotorReduction =
      compRobot ? FlexConstants.turnMotorReduction : SDSConstants.turnMotorReduction;
  public static final DCMotor turnGearbox =
      compRobot ? FlexConstants.turnGearbox : SDSConstants.turnGearbox;

  // Turn encoder configuration
  public static final boolean turnEncoderInverted =
      compRobot ? FlexConstants.turnEncoderInverted : SDSConstants.turnEncoderInverted;
  public static final double turnEncoderPositionFactor =
      2 * Math.PI / turnMotorReduction; // Rotor Rotations -> Radians
  public static final double turnEncoderVelocityFactor =
      turnEncoderPositionFactor / 60.0; // Rotor Meters -> Radians/Sec

  public static final double absoluteTurnEncoderPositionFactor =
      compRobot ? FlexConstants.absoluteTurnEncoderPositionFactor : -1;
  public static final double absoluteTurnEncoderVelocityFactor =
      compRobot ? FlexConstants.absoluteTurnEncoderVelocityFactor : -1;

  // Turn PID configuration
  public static final double turnKp = compRobot ? FlexConstants.turnKp : SDSConstants.turnKp;
  public static final double turnKd = compRobot ? FlexConstants.turnKd : SDSConstants.turnKd;

  public static final double turnSimP = compRobot ? FlexConstants.turnSimP : SDSConstants.turnSimP;
  public static final double turnSimD = compRobot ? FlexConstants.turnSimD : SDSConstants.turnSimD;

  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg =
      compRobot ? FlexConstants.robotMassKg : SDSConstants.robotMassKg;
  public static final double robotMOI = compRobot ? FlexConstants.robotMOI : SDSConstants.robotMOI;
  public static final double wheelCOF = compRobot ? FlexConstants.wheelCOF : SDSConstants.wheelCOF;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);
}
