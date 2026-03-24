package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.util.FuelVelocity;
import java.util.Arrays;

public class AutoAlignJiggle extends InstantCommand {
  private Drive driveRef;

  private PIDController thetaControllerRobot = Constants.thetaController;

  private boolean aligned = false;

  private double[][] jiggleStates = {{1, 1}, {-1, 1}, {-1, -1}, {1, -1}};
  private int jiggleModulo = 10;
  private int jiggleIndex = 0;
  private int jiggleCount = 0;
  private double xIn = 0;
  private double yIn = 0;

  private ChassisSpeeds maximumSpeeds = new ChassisSpeeds(6.035, 6.035, Math.PI * 3);
  private Translation3d goalErrors = new Translation3d(0.05, 0.05, Math.PI / 720);

  private Superstructure superstructure;

  public AutoAlignJiggle(Drive drive, Superstructure superstructure) {
    driveRef = drive;
    this.superstructure = superstructure;
  }

  @Override
  public void initialize() {
    superstructure.useCalcVelocity = true;
    jiggleCount = 0;
    jiggleIndex = 0;
  }

  @Override
  public void execute() {
    aligned = runRobotRelativeAlign(maximumSpeeds, goalErrors);
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.useCalcVelocity = false;
  }

  @Override
  public boolean isFinished() {
    if (aligned) {
      // RobotContainer.addAlert(new AlertBody(AlertMode.FULLY_ALIGNED, 0.5));
    }
    return aligned;
  }

  public boolean runRobotRelativeAlign(ChassisSpeeds maximumSpeeds, Translation3d goalErrors) {

    double maxSpeedTheta = Math.abs(maximumSpeeds.omegaRadiansPerSecond);

    Pose2d hubPosition = driveRef.getPose().nearest(Arrays.asList(FuelVelocity.HUB_POSITION));
    double yDif = driveRef.getPose().getY() - hubPosition.getY();
    double xDif = driveRef.getPose().getX() - hubPosition.getX();
    double calcTheta = Math.atan2(yDif, xDif);

    Pose2d fieldRelativeTarget =
        new Pose2d(driveRef.getPose().getX(), driveRef.getPose().getY(), new Rotation2d(calcTheta));

    Pose2d offsetPose = driveRef.getPose().relativeTo(fieldRelativeTarget);

    jiggleCount++;

    if (jiggleCount % jiggleModulo == 0) {
      xIn = jiggleStates[jiggleIndex % jiggleStates.length][0];
      yIn = jiggleStates[jiggleIndex % jiggleStates.length][1];
      jiggleIndex++;
    }

    System.out.println(xIn + ", " + yIn);

    Translation2d linearVelocity = DriveCommands.getLinearVelocityFromJoysticks(xIn, yIn);
    double thetaAlignSpeed =
        MathUtil.clamp(
            thetaControllerRobot.calculate(offsetPose.getRotation().getRadians()),
            -maxSpeedTheta,
            maxSpeedTheta);
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * driveRef.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * driveRef.getMaxLinearSpeedMetersPerSec(),
            0);

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped
                ? driveRef.getRotation().plus(new Rotation2d(Math.PI))
                : driveRef.getRotation());

    if (Math.abs(offsetPose.getRotation().getRadians()) < goalErrors.getZ()) {
      thetaAlignSpeed = 0.0;
    }

    speeds = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, thetaAlignSpeed);

    driveRef.runVelocity(speeds);

    return false;
  }
}
