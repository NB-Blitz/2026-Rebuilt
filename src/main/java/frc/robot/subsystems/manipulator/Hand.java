package frc.robot.subsystems.manipulator;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;

public class Hand implements HandInterface {

  private final int topMotorCANID = 13;
  private final int bottomMotorCANID = 14;
  private final int coralSensorID = 1;
  private final int currentLimit = 80;

  private final double percent = 0.3;
  private final double coralIntakeSpeed = 0.3;
  private final double coralExpelSpeed = -0.3;
  private final double algaeIntakeSpeed = -0.3;
  private final double algaeExpelNetSpeed = 0.3;
  private final double algaeExpelProcessorSpeed = 0.3;

  private final boolean motorInverted = true;

  private final SparkBase topMotor;
  private final SparkBase bottomMotor;

  private final DigitalInput coralSensor;

  public Hand() {

    topMotor = new SparkFlex(topMotorCANID, MotorType.kBrushless);
    bottomMotor = new SparkFlex(bottomMotorCANID, MotorType.kBrushless);

    coralSensor = new DigitalInput(coralSensorID);

    SparkFlexConfig bottomMotorConfig = new SparkFlexConfig();
    bottomMotorConfig
        .idleMode(IdleMode.kBrake)
        .inverted(motorInverted)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(12.0);
    bottomMotorConfig
        .signals
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        bottomMotor,
        5,
        () ->
            bottomMotor.configure(
                bottomMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkFlexConfig topMotorConfig = new SparkFlexConfig();
    topMotorConfig
        .idleMode(IdleMode.kBrake)
        .inverted(!motorInverted)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(12.0);
    topMotorConfig
        .signals
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        topMotor,
        5,
        () ->
            topMotor.configure(
                topMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public boolean holdingCoral() {
    return !coralSensor.get();
  }

  // Coral in = algae out
  // Coral out = algae in

  public void clockwise() {
    topMotor.set(percent);
    bottomMotor.set(percent);
  }

  public void counterClockwise() {
    topMotor.set(-percent);
    bottomMotor.set(-percent);
  }

  public void stopMotors() {
    topMotor.set(0);
    bottomMotor.set(0);
  }

  public void intakeCoral() {
    topMotor.set(coralIntakeSpeed);
    bottomMotor.set(coralIntakeSpeed);
  }

  public void expelCoral() {
    topMotor.set(coralExpelSpeed);
    bottomMotor.set(coralExpelSpeed);
  }

  public void expelCoralAuto() {
    topMotor.set(coralExpelSpeed * 0.5);
    bottomMotor.set(coralExpelSpeed * 0.5);
  }

  public void intakeAlgae() {
    topMotor.set(algaeIntakeSpeed);
    bottomMotor.set(0);
  }

  public void expelAlgaeNet() {
    topMotor.set(algaeExpelNetSpeed);
    bottomMotor.set(0);
  }

  public void expelAlgaeProcessor() {
    topMotor.set(algaeExpelProcessorSpeed);
    bottomMotor.set(0);
  }
}
