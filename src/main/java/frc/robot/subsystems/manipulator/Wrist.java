package frc.robot.subsystems.manipulator;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.subsystems.common.Joint;
import org.littletonrobotics.junction.Logger;

public class Wrist extends Joint {

  private static final double jointP = 0.01;
  private static final double jointI = 0.0;
  private static final double jointD = 0.0;
  private static final double jointFF = 0.0;
  private static final double gearRatio = 1 / 90.0;
  private static final int jointMotorCANID = 12;
  private static final double maxJointSpeed = 90.0; // degrees per second
  private static final double maxAcceleration = maxJointSpeed * 1.5;
  private static final double allowedError = 1;

  // create an enum for preset elevator heights (ex. coral level 1, 2, 3, 4)
  public enum WristAngle {
    bottom(152),
    algaeProcessor(149),
    coralStation(135),
    coralL1(147),
    frontIntake(150),
    coralL2(115),
    algaeInReefL2(115),
    coralL3(91),
    algaeInReefL3(115),
    coralL4(120),
    algaeBarge(25);

    public final double angle;

    WristAngle(double angle) {
      this.angle = angle;
    }
  }

  public Wrist() {
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
        false,
        true,
        152.0,
        25,
        75.0,
        new SparkFlex(jointMotorCANID, MotorType.kBrushless),
        new SparkFlexConfig());

    super.noFoulPos = 66;
  }

  @Override
  public void setJointAngle(int enumIndex) {
    super.endTargetAngle = WristAngle.values()[enumIndex].angle + angleOffset;
    super.controlMode = "preset";
  }

  @Override
  public void updateJoint() {
    super.updateJoint();
    Logger.recordOutput("Manipulator/Wrist/Angle", getPosition());
    Logger.recordOutput("Manipulator/Wrist/Current", jointMotor.getOutputCurrent());
    Logger.recordOutput("Manipulator/Wrist/Target Angle", super.targetAngle - angleOffset);
  }
}
