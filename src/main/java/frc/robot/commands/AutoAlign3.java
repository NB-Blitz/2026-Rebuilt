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
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class AutoAlign3 extends InstantCommand {
  private Drive driveRef;
  private DoubleSupplier xJoystick;
  private DoubleSupplier yJoystick;

  private PIDController thetaControllerRobot = Constants.thetaController;

  private boolean aligned = false;

  public static boolean hubAutoAligning = false;
  public static double thetaSpeed = 0.0;

  private ChassisSpeeds maximumSpeeds = new ChassisSpeeds(6.035, 6.035, Math.PI * 3);
  private Translation3d goalErrors = new Translation3d(0.05, 0.05, Math.PI / 720);

  private Superstructure superstructure;

  public AutoAlign3(
      Drive drive,
      Superstructure superstructure,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    driveRef = drive;
    this.superstructure = superstructure;
    xJoystick = xSupplier;
    yJoystick = ySupplier;
  }

  @Override
  public void initialize() {
    superstructure.useCalcVelocity = true;
    hubAutoAligning = true;
  }

  @Override
  public void execute() {
    aligned = runRobotRelativeAlign(maximumSpeeds, goalErrors);
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.useCalcVelocity = false;
    hubAutoAligning = false;
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

    Translation2d linearVelocity;
    if (AutoAlignJiggle.jiggling) {
      linearVelocity =
          DriveCommands.getLinearVelocityFromJoysticks(AutoAlignJiggle.xIn, AutoAlignJiggle.yIn);
    } else {
      linearVelocity =
          DriveCommands.getLinearVelocityFromJoysticks(
              xJoystick.getAsDouble(), yJoystick.getAsDouble());
    }
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

    double diff = Math.abs(offsetPose.getRotation().getRadians());
    Logger.recordOutput("AutoAlign3/Theta Diff", diff);
    if (diff < goalErrors.getZ()) {
      thetaAlignSpeed = 0.0;
    }

    thetaSpeed = thetaAlignSpeed;
    speeds = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, thetaAlignSpeed);

    driveRef.runVelocity(speeds);

    // return thetaAlignSpeed == 0.0;
    return false;
  }
}
