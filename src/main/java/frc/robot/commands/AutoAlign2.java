package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FuelVelocity;

public class AutoAlign2 extends InstantCommand {
  private Drive driveRef;

  private ChassisSpeeds speeds = new ChassisSpeeds();

  private PIDController thetaControllerRobot = Constants.thetaController;

  private boolean aligned = false;

  private ChassisSpeeds maximumSpeeds = new ChassisSpeeds(1.25, 1.25, 1.85);
  private Translation3d goalErrors = new Translation3d(0.01, 0.01, 0.01);

  public AutoAlign2(Drive drive) {
    driveRef = drive;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    aligned = runRobotRelativeAlign(maximumSpeeds, goalErrors);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (aligned) {
      // RobotContainer.addAlert(new AlertBody(AlertMode.FULLY_ALIGNED, 0.5));
    }
    return aligned;
  }

  public boolean runRobotRelativeAlign(ChassisSpeeds maximumSpeeds, Translation3d goalErrors) {

    double maxSpeedTheta = Math.abs(maximumSpeeds.omegaRadiansPerSecond);

    double yDif = driveRef.getPose().getY() - FuelVelocity.HUB_POSITION.getY();
    double xDif = driveRef.getPose().getX() - FuelVelocity.HUB_POSITION.getX();
    double calcTheta = Math.atan2(yDif, xDif);

    Pose2d fieldRelativeTarget =
        new Pose2d(driveRef.getPose().getX(), driveRef.getPose().getY(), new Rotation2d(calcTheta));

    Pose2d offsetPose = driveRef.getPose().relativeTo(fieldRelativeTarget);

    double xAlignSpeed = 0;
    double yAlignSpeed = 0;
    double thetaAlignSpeed =
        MathUtil.clamp(
            thetaControllerRobot.calculate(offsetPose.getRotation().getRadians()),
            -maxSpeedTheta,
            maxSpeedTheta);

    if (Math.abs(offsetPose.getX()) < goalErrors.getX()) {
      xAlignSpeed = 0.0;
    }

    if (Math.abs(offsetPose.getY()) < goalErrors.getY()) {
      yAlignSpeed = 0.0;
    }

    if (Math.abs(offsetPose.getRotation().getRadians()) < goalErrors.getZ()) {
      thetaAlignSpeed = 0.0;
    }

    speeds = new ChassisSpeeds(xAlignSpeed, yAlignSpeed, thetaAlignSpeed);

    driveRef.runVelocity(speeds);

    return xAlignSpeed == 0.0 && yAlignSpeed == 0.0 && thetaAlignSpeed == 0.0;
  }
}
