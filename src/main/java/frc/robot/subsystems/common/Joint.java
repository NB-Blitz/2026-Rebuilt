package frc.robot.subsystems.common;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.Timer;

public class Joint {

  protected String controlMode = "manual";
  protected double endTargetAngle = 0;
  protected double targetAngle = 0;

  protected SparkBase jointMotor;
  private RelativeEncoder jointEncoder;
  private AbsoluteEncoder jointAbsoluteEncoder;
  private SparkClosedLoopController jointController;
  private final int currentLimit = 300;
  private final double maxJointSpeed;
  private final double angleIncrement;
  private final double topLimit;
  private final double bottomLimit;
  protected final double angleOffset;
  private final double kAngleTolerance =
      1.25; // If the relaive encoder is plus or minus this value it sets it to the absolute tl;dr
  // fixes belt skipping

  public double noFoulPos;

  public Joint(
      double P,
      double I,
      double D,
      double FF,
      double gearRatio,
      double maxSpeed,
      double maxAcceleration,
      double allowedError,
      double absGearRatio,
      boolean absInvert,
      boolean invert,
      double kForwardSoftLimit,
      double kReverseSoftLimit,
      double angleOffsettywettyfetty,
      SparkBase motorRef,
      SparkBaseConfig config) {
    jointMotor = motorRef;
    jointController = jointMotor.getClosedLoopController();
    maxJointSpeed = maxSpeed;
    angleIncrement = maxJointSpeed / 50;
    angleOffset = angleOffsettywettyfetty;
    topLimit = kForwardSoftLimit + angleOffset;
    bottomLimit = kReverseSoftLimit + angleOffset;

    jointEncoder = jointMotor.getEncoder();
    jointAbsoluteEncoder = jointMotor.getAbsoluteEncoder();

    var jointConfig = config;

    jointConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
    jointConfig.inverted(invert);
    jointConfig
        .encoder
        .positionConversionFactor(gearRatio * 360)
        .velocityConversionFactor(gearRatio * 6)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    jointConfig
        .absoluteEncoder
        .inverted(absInvert)
        .positionConversionFactor(absGearRatio * 360)
        .velocityConversionFactor(absGearRatio * 6)
        .averageDepth(2);
    jointConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(
            P, I,
            D, FF)
        .positionWrappingEnabled(true)
        .maxMotion
        .maxVelocity(maxSpeed)
        .maxAcceleration(maxAcceleration)
        .allowedClosedLoopError(allowedError);
    jointConfig
        .softLimit
        .forwardSoftLimit(kForwardSoftLimit + angleOffset)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(kReverseSoftLimit + angleOffset)
        .reverseSoftLimitEnabled(true);
    jointConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(10)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs(10)
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        jointMotor,
        5,
        () ->
            jointMotor.configure(
                jointConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(jointMotor, 5, () -> jointEncoder.setPosition(jointAbsoluteEncoder.getPosition()));

    Timer.delay(0.1);
    targetAngle = jointEncoder.getPosition();
  }

  public Joint(
      double P,
      double I,
      double D,
      double FF,
      double gearRatio,
      double maxSpeed,
      boolean invert,
      SparkBase motorRef,
      SparkBaseConfig config) {
    angleOffset = 0.0;
    topLimit = 360;
    bottomLimit = 0;
    jointMotor = motorRef;
    jointController = jointMotor.getClosedLoopController();
    this.maxJointSpeed = maxSpeed;
    angleIncrement = maxJointSpeed / 50;
    jointEncoder = jointMotor.getEncoder();
    jointAbsoluteEncoder = null;

    var jointConfig = config;
    jointConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
    jointConfig.inverted(invert);
    jointConfig
        .encoder
        .positionConversionFactor(gearRatio * 360)
        .velocityConversionFactor(gearRatio * 6)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    jointConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(
            P, I,
            D, FF)
        .positionWrappingEnabled(true)
        .maxMotion
        .maxVelocity(0)
        .maxAcceleration(0)
        .allowedClosedLoopError(0);
    jointConfig
        .softLimit
        .forwardSoftLimit(180) // TODO update max height in meters
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(0)
        .reverseSoftLimitEnabled(true);
    jointConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(10)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        jointMotor,
        5,
        () ->
            jointMotor.configure(
                jointConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(jointMotor, 5, () -> jointEncoder.setPosition(0));

    Timer.delay(0.1);
    targetAngle = jointEncoder.getPosition();
  }

  public Joint() {
    angleOffset = 0.0;
    maxJointSpeed = 0;
    angleIncrement = 0;
    topLimit = 0;
    bottomLimit = 0;
  }

  public void resetEncoder(double angle) {
    jointEncoder.setPosition(angle + angleOffset);
  }

  public double getPosition() {
    return jointEncoder.getPosition() - angleOffset;
  }

  public void setJointSpeed(double joystickInput) {
    targetAngle += joystickInput * angleIncrement;
    controlMode = "manual";
  }

  public void setJointAngle(int enumIndex) {
    targetAngle = enumIndex + angleOffset;
    controlMode = "preset";
  }

  public void setJointAngleRaw(double angle) {
    endTargetAngle = angle + angleOffset;
    controlMode = "preset";
  }

  public void eStop() {
    controlMode = "estop";
  }

  public void resetTargetAngle() {
    targetAngle = jointEncoder.getPosition();
    controlMode = "manual";
  }

  public double getTarget() {
    if (controlMode == "preset") {
      return endTargetAngle - angleOffset;
    }
    return targetAngle - angleOffset;
  }

  public void updateJoint() {
    if (jointAbsoluteEncoder != null) {
      if (Math.abs(jointAbsoluteEncoder.getPosition() - jointEncoder.getPosition())
          > kAngleTolerance) {
        tryUntilOk(
            jointMotor, 5, () -> jointEncoder.setPosition(jointAbsoluteEncoder.getPosition()));
      }
    }

    if (controlMode == "preset") {
      double targetDiff = endTargetAngle - targetAngle;
      double diffAbs = Math.abs(targetDiff);
      if (diffAbs > angleIncrement) {
        if (targetDiff > 0) {
          targetAngle += angleIncrement;
        } else if (targetDiff < 0) {
          targetAngle -= angleIncrement;
        }
      } else {
        targetAngle = endTargetAngle;
      }
    }

    if (targetAngle < bottomLimit) {
      targetAngle = bottomLimit;
    } else if (targetAngle > topLimit) {
      targetAngle = topLimit;
    }

    if (controlMode == "estop") {
      jointController.setReference(0, ControlType.kDutyCycle);
    } else {
      jointController.setReference(targetAngle, ControlType.kPosition);
    }
  }
}
