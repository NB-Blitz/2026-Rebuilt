package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

public class FuelVelocity {
  public static final double HUB_HEIGHT = 72.0; // in inches
  public static final double ROBOT_SHOOTER_HEIGHT =
      18.309; // in inches FIXME: get right measurement
  public static final double THETA = Units.degreesToRadians(68.03); // FIXME: launch angle
  public static final double GRAVITY =
      9.80665000; // most accurate numbers FIXME: make it more accurate up to 100 decimals
  public static final double HEIGHT = Units.inchesToMeters(HUB_HEIGHT - ROBOT_SHOOTER_HEIGHT);
  public static final Translation2d HUB_DIMENTIONS = new Translation2d(44.4, 44.4); // in inches
  public static final Pose2d HUB_POSITION =
      new Pose2d(
          new Translation2d(4.625, 4.035),
          new Rotation2d(0.0)); // FIXME: plug in the center of the hub here in meters
  public static final Transform2d SHOOTER_POSITION =
      new Transform2d(
          Units.inchesToMeters(8.299),
          0.0,
          new Rotation2d(Math.PI)); // FIXME: get correct x value for the shooter offset
  public static final Transform2d SHOOTER_POSITION_INV =
      new Transform2d(
          Units.inchesToMeters(-8.299),
          0.0,
          new Rotation2d(Math.PI)); // FIXME: get correct x value for the shooter offset

  /**
   * Calculates the distance from the center of the robot's shooter to the center of the hub given
   * the robots position.
   *
   * @param robotPosition - the position of the robot as a Pose2D
   * @return - linear distance in meters
   */
  public static double calcPosFromHub(Pose2d robotPosition) {
    Pose2d fieldRelativeShooter = robotPosition.transformBy(SHOOTER_POSITION);
    Transform2d posDifference = HUB_POSITION.minus(fieldRelativeShooter);
    return Math.hypot(posDifference.getX(), posDifference.getY());
  }

  /**
   * Calculates the required initial launch velocity of the robot from a fixed point given the
   * robot's position. Equation is v = sqrt(gd^2 / (2cos(a)^2 * (d * tan(a) - h))) this took too
   * long to derive
   *
   * @param robotPostion - the position of the robot as a Pose2D
   * @return returns the velocity in meters per second
   */
  public static double calcFixedLaunchVelocity(Pose2d robotPosition) {
    double result = 0.0;
    double distanceFromHub = calcPosFromHub(robotPosition);

    Logger.recordOutput("FuelVelocity/DistanceFromHub", distanceFromHub);

    result = GRAVITY * Math.pow(distanceFromHub, 2);
    result =
        result / (2 * Math.pow(Math.cos(THETA), 2) * (distanceFromHub * Math.tan(THETA) - HEIGHT));
    result = Math.sqrt(result);

    Logger.recordOutput("FuelVelocity/Velocity", result);

    return result;
  }
}
