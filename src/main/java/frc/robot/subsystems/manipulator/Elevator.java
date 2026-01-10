package frc.robot.subsystems.manipulator;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class Elevator implements ElevatorInterface {

  // creating all the constants

  public static final int kLeadMotorCANID = 9;
  public static final int kFollowMotorCANID = 10;

  public static final double kWheelDiameter = 0.045; // in meters
  public static final double kWheelCircumference = Math.PI * kWheelDiameter; // in meters
  public static final double kGearRatio = 1 / 5.0;
  public static final double kRotationSpeed = 1.0;
  public static final IdleMode kMotorIdleMode = IdleMode.kBrake;
  public static final int kMotorCurrentLimit = 140;

  // these are values for the PID controller
  public static final double kP = 4.5; // MAXMotion: 0.5
  public static final double kI = 0.0005; // MAXMotion: 0
  public static final double kD = 0.0;
  public static final double kFF = 0.0;

  private String controlMode = "manual";
  private double endTargetPos = 0;
  private double targetPosition = 0;

  public static final double kPositionConversionFactor =
      kGearRatio * kWheelCircumference; // in meters
  public static final double kVelocityConversionFactor =
      kPositionConversionFactor / 60.0; // in meters per second

  // the left motor is turning the opposite direction, the right motor is not
  public static final boolean kLeadInverted = false;

  // create the motors, one on the left side of the elevator and one on the right side
  // we need two motors to provide enough power to move the elevator quickly
  private final SparkBase m_followMotor = new SparkFlex(kFollowMotorCANID, MotorType.kBrushless);
  private final SparkBase m_leadMotor = new SparkFlex(kLeadMotorCANID, MotorType.kBrushless);

  // create the relative encoders (one for each motor)
  // they track the position of the motors
  private final RelativeEncoder m_leadEncoder = m_leadMotor.getEncoder();
  // private final AbsoluteEncoder m_leadAbsEncoder = m_leadMotor.getAbsoluteEncoder();

  // set the input for the elevator
  private final SparkLimitSwitch m_bottomSwitch = m_leadMotor.getReverseLimitSwitch();
  // create the PID controller (only for the left motor)
  private final SparkClosedLoopController m_PIDController = m_leadMotor.getClosedLoopController();

  private final double maxElevatorSpeed = 0.7; // meters per second
  private final double maxAcceleration = maxElevatorSpeed / 4;
  private final double allowedError = 0.001;
  private final double positionIncrement = maxElevatorSpeed / 50;
  private final double topLimit = 0.75;
  private final double bottomLimit = 0;

  public Elevator() {

    // set up Spark Flex configuration for the left motor
    // (not sure what everything is set to but I think it works)
    SparkFlexConfig leadMotorConfig = new SparkFlexConfig();
    leadMotorConfig.inverted(kLeadInverted);
    leadMotorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(kMotorCurrentLimit)
        .voltageCompensation(12.0);
    leadMotorConfig
        .encoder
        .positionConversionFactor(kPositionConversionFactor)
        .velocityConversionFactor(kVelocityConversionFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    leadMotorConfig
        .absoluteEncoder
        .inverted(false)
        .positionConversionFactor(kPositionConversionFactor * 5)
        .velocityConversionFactor(kVelocityConversionFactor * 5)
        .averageDepth(2);
    leadMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(kP, kI, kD, kFF)
        .maxMotion
        .maxVelocity(maxElevatorSpeed * 60 / kPositionConversionFactor)
        .maxAcceleration(maxAcceleration * 60 / kPositionConversionFactor)
        .allowedClosedLoopError(allowedError);
    leadMotorConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs(10)
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    leadMotorConfig
        .limitSwitch
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(true);
    leadMotorConfig.softLimit.forwardSoftLimit(topLimit).forwardSoftLimitEnabled(true);

    // sets the configuration of the right motor
    tryUntilOk(
        m_leadMotor,
        5,
        () ->
            m_leadMotor.configure(
                leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // resets the right encoder position to 0.0
    // tryUntilOk(m_leadMotor, 5, () -> m_leadEncoder.setPosition(m_leadAbsEncoder.getPosition()));

    Timer.delay(0.1);
    targetPosition = getHeight();

    // set up Spark Flex configuration for the left motor
    // (not sure what everything is set to but I think it works)
    SparkFlexConfig followMotorConfig = new SparkFlexConfig();
    followMotorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(kMotorCurrentLimit)
        .voltageCompensation(12.0);
    followMotorConfig
        .signals
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    followMotorConfig.follow(kLeadMotorCANID, true);

    // sets the configuration of the left motor
    tryUntilOk(
        m_followMotor,
        5,
        () ->
            m_followMotor.configure(
                followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  // returns the state of the limit switch
  public boolean getLimit() {
    return m_bottomSwitch.isPressed();
  }

  // moves the elevator to a preset position specified by the Position parameter (created in the
  // enum)
  public void setPosition(ElevatorPosition position) {
    endTargetPos = position.position;
    controlMode = "preset";
  }

  public void setSpeed(double joystickInput) {
    targetPosition += joystickInput * positionIncrement;
    controlMode = "manual";
  }

  public void eStop() {
    controlMode = "estop";
  }

  // moves the elevator a certain speed according to the double parameter
  public void move() {
    if (getLimit()) {
      tryUntilOk(m_leadMotor, 5, () -> m_leadEncoder.setPosition(bottomLimit));
      if (targetPosition < bottomLimit) {
        targetPosition = bottomLimit;
      }
    }

    if (controlMode == "preset") {
      double targetDiff = endTargetPos - targetPosition;
      double diffAbs = Math.abs(targetDiff);
      if (diffAbs > positionIncrement) {
        if (targetDiff > 0) {
          targetPosition += positionIncrement;
        } else if (targetDiff < 0) {
          targetPosition -= positionIncrement;
        }
      } else {
        targetPosition = endTargetPos;
      }
    }

    if (targetPosition > topLimit) {
      targetPosition = topLimit;
    }

    if (controlMode == "estop") {
      m_PIDController.setReference(0, ControlType.kDutyCycle);
    } else {
      m_PIDController.setReference(targetPosition, ControlType.kPosition);
    }

    Logger.recordOutput("Manipulator/Elevator/Height", getHeight());
    Logger.recordOutput("Manipulator/Elevator/Target Height", targetPosition);
  }

  public double getHeight() {
    return m_leadEncoder.getPosition();
  }

  public void resetTargetHeight() {
    targetPosition = getHeight();
    controlMode = "manual";
  }

  public double getTarget() {
    if (controlMode == "preset") {
      return endTargetPos;
    }
    return targetPosition;
  }

  public double getTopLimit() {
    return topLimit;
  }
}
