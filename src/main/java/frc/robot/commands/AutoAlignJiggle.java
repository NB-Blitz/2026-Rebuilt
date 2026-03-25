package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Drive;

public class AutoAlignJiggle extends InstantCommand {

  private Drive driveRef;

  public static boolean jiggling = false;

  private double[][] jiggleStates = {{0.7, 0.7}, {-0.7, 0.7}, {-0.7, -0.7}, {0.7, -0.7}};
  private int jiggleModulo = 10;
  private int jiggleIndex = 0;
  private int jiggleCount = 0;

  // joystick input
  public static double xIn = 0;
  public static double yIn = 0;

  public AutoAlignJiggle(Drive driveRef) {
    this.driveRef = driveRef;
  }

  @Override
  public void initialize() {
    jiggleCount = 0;
    jiggleIndex = 0;
    jiggling = true;
    //System.out.println("IT HAS BEGUN" + jiggling);
  }

  @Override
  public void execute() {
    runJiggle();
  }

  @Override
  public void end(boolean interrupted) {
    jiggling = false;
    //System.out.println("ITS THE END" + jiggling);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public void runJiggle() {

    jiggleCount++;

    // System.out.println(jiggleCount + ", " + jiggleIndex + " JIGGLE
    // STUFFFFFFFFFFFFFFFFFFFFFFFFFFFF");

    if (jiggleCount % jiggleModulo == 0) {
      xIn = jiggleStates[jiggleIndex % jiggleStates.length][0];
      yIn = jiggleStates[jiggleIndex % jiggleStates.length][1];
      jiggleIndex++;
    }

    // System.out.println(xIn + ", " + yIn);

    Translation2d linearVelocity = DriveCommands.getLinearVelocityFromJoysticks(xIn, yIn);
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * driveRef.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * driveRef.getMaxLinearSpeedMetersPerSec(),
            0);
    speeds =
        new ChassisSpeeds(
            speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, AutoAlign3.thetaSpeed);

    driveRef.runVelocity(speeds);
  }
}
