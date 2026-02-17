package frc.robot.subsystems.drive.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class SDSConstants {
  public static final double maxSpeedMetersPerSec = 3.81;
  public static final double maxAcceleration = 3.0;
  public static final double allowedError = 0.03;
  public static final double trackWidth = Units.inchesToMeters(18.5);
  public static final double wheelBase = Units.inchesToMeters(19.5);

  // Zeroed rotation values for each module
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(-0.439 * 2 * Math.PI);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(-0.331 * 2 * Math.PI);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(0.063 * 2 * Math.PI);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(0.304 * 2 * Math.PI);

  // Device CAN IDs
  public static final int flTurningEncoderCANId = 9;
  public static final int frTurningEncoderCANId = 10;
  public static final int blTurningEncoderCANId = 11;
  public static final int brTurningEncoderCANId = 12;

  // Drive motor configuration
  public static final boolean driveInverted = true;
  public static final int driveMotorCurrentLimit = 50;
  public static final double wheelRadiusMeters = Units.inchesToMeters(2);
  public static final double driveMotorReduction =
      8.14; // MAXSwerve with 14 pinion teeth and 22 spur teeth
  public static final DCMotor driveGearbox = DCMotor.getNEO(1);

  // Drive PID configuration
  public static final double driveKp = 0.01;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.10346;
  public static final double driveKv = 0.15953;

  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 50;
  public static final double turnMotorReduction = 12.8;
  public static final DCMotor turnGearbox = DCMotor.getNEO(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;

  // Turn PID configuration
  public static final double turnKp = 0.45;
  public static final double turnKd = 0.0;

  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;

  // PathPlanner configuration
  public static final double robotMassKg = 29.55; // SDS
  public static final double robotMOI = 4.0; // Estimated for SDS
  public static final double wheelCOF = 1.2; // Tread, Colson wheel:1.0
}
