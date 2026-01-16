package frc.robot.subsystems.manipulator;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.subsystems.common.Joint;
import org.littletonrobotics.junction.Logger;

public class Shoulder extends Joint {

  private static final double jointP = 0.02;
  private static final double jointI = 0.0;
  private static final double jointD = 0.0;
  private static final double jointFF = 0.0;
  private static final double gearRatio = 1 / (64 * 3.6);
  private static final int jointMotorCANID = 11;
  private static final double maxJointSpeed = 120.0; // degrees per second
  private static final double maxAcceleration = maxJointSpeed * 1.5;
  private static final double allowedError = 1;
  private static final double forwardLimit = 180.0;
  private static final double reverseLimit = 0;
  private static final double angleOffset = 7;

  // create an enum for preset elevator heights (ex. coral level 1, 2, 3, 4)
  public enum ShoulderAngle {
    bottom(0),
    algaeProcessor(0),
    coralStation(0),
    coralL1(12),
    frontIntake(53.6),
    coralL2(45),
    algaeInReefL2(45),
    coralL3(66.5),
    algaeInReefL3(45),
    coralL4(150),
    algaeBarge(175);

    public final double angle;

    ShoulderAngle(double angle) {
      this.angle = angle;
    }
  }

  public Shoulder() {
    super(
        jointP,
        jointI,
        jointD,
        jointFF,
        gearRatio,
        maxJointSpeed,
        maxAcceleration,
        allowedError,
        1.0,
        true,
        true,
        forwardLimit,
        reverseLimit,
        angleOffset,
        new SparkFlex(jointMotorCANID, MotorType.kBrushless),
        new SparkFlexConfig());

    super.noFoulPos = 95.5;
  }

  @Override
  public void setJointAngle(int enumIndex) {
    super.endTargetAngle = ShoulderAngle.values()[enumIndex].angle + angleOffset;
    super.controlMode = "preset";
  }

  @Override
  public void updateJoint() {
    super.updateJoint();
    Logger.recordOutput("Manipulator/Shoulder/Angle", getPosition());
    Logger.recordOutput("Manipulator/Shoulder/Current", jointMotor.getOutputCurrent());
    Logger.recordOutput("Manipulator/Shoulder/Target Angle", targetAngle - angleOffset);
  }
}
