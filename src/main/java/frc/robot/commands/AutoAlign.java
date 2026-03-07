package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class AutoAlign extends InstantCommand {
  private Drive driveRef;
  private List<Pose2d> tagPoseList = new ArrayList<>();
  private Transform2d[] offsetFromTag;
  private int offsetIndex = 0;
  private boolean ignoreRotation;

  private ChassisSpeeds speeds = new ChassisSpeeds();

  private PIDController xControllerRobot = Constants.xController;
  private PIDController yControllerRobot = Constants.yController;
  private PIDController thetaControllerRobot = Constants.thetaController;

  private boolean aligned = false;

  private ChassisSpeeds maximumSpeeds = new ChassisSpeeds(4.5, 4.5, Math.PI * 1.5);
  private Translation3d goalErrors =
      new Translation3d(0.075, 0.075, Math.PI / 90); // 0.125, 0.075, pi/90

  private Pose2d cachedTarget = null;
  private Pose2d cachedTag = null;

  public AutoAlign(Drive drive, int[] Tags, Transform2d[] offsetPose2d, boolean ignoreRotation) {
    driveRef = drive;
    for (int tagId : Tags) {
      Optional<Pose3d> tagPose = VisionConstants.aprilTagLayout.getTagPose(tagId);
      if (tagPose.isPresent()) {
        tagPoseList.add(tagPose.get().toPose2d());
      }
    }
    offsetFromTag = offsetPose2d;
    this.ignoreRotation = ignoreRotation;
  }

  @Override
  public void initialize() {
    cachedTarget = null;
    cachedTag = null;
    offsetIndex = 0;
  }

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

    double maxSpeedX = Math.abs(maximumSpeeds.vxMetersPerSecond);
    double maxSpeedY = Math.abs(maximumSpeeds.vyMetersPerSecond);
    double maxSpeedTheta = Math.abs(maximumSpeeds.omegaRadiansPerSecond);

    if (tagPoseList.isEmpty() && cachedTarget == null) {
      return true;
    }
    Pose2d fieldRelativeTarget;
    if (cachedTag == null) {
      cachedTag = driveRef.getPose().nearest(tagPoseList);
    }
    if (cachedTarget == null) {
      fieldRelativeTarget = cachedTag.transformBy(offsetFromTag[offsetIndex]);
      cachedTarget = fieldRelativeTarget;
    } else {
      fieldRelativeTarget = cachedTarget;
    }

    // Pose2d offsetPose = driveRef.getPose().relativeTo(fieldRelativeTarget);
    Pose2d offsetPose = fieldRelativeTarget.relativeTo(driveRef.getPose());

    double xAlignSpeed =
        MathUtil.clamp(-1 * xControllerRobot.calculate(offsetPose.getX()), -maxSpeedX, maxSpeedX);
    double yAlignSpeed =
        MathUtil.clamp(-1 * yControllerRobot.calculate(offsetPose.getY()), -maxSpeedY, maxSpeedY);
    double thetaAlignSpeed =
        MathUtil.clamp(
            -thetaControllerRobot.calculate(offsetPose.getRotation().getRadians()),
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

    if (ignoreRotation == true) {
      thetaAlignSpeed = 0.0;
    }

    speeds = new ChassisSpeeds(xAlignSpeed, yAlignSpeed, thetaAlignSpeed);

    driveRef.runVelocity(speeds);

    if (xAlignSpeed == 0.0 && yAlignSpeed == 0.0 && thetaAlignSpeed == 0.0) {
      cachedTarget = null;
      offsetIndex++;
    }
    return offsetIndex >= offsetFromTag.length;
  }
}
