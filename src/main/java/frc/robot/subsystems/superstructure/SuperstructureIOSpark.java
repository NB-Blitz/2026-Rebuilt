// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.superstructure;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * This superstructure implementation is for Spark devices. It defaults to brushless control, but
 * can be easily adapted for a brushed motor. One or more Spark Flexes can be used by swapping
 * relevant instances of "SparkFlex" with "SparkFlex".
 */
public class SuperstructureIOSpark implements SuperstructureIO {
  private final SparkFlex feeder = new SparkFlex(feederCanId, MotorType.kBrushless);
  private final SparkFlex launcher = new SparkFlex(launcherCanId, MotorType.kBrushless);
  // private final SparkFlex launcherFollower = new SparkFlex(launcherFollowerCanId,
  // MotorType.kBrushless);
  private final SparkFlex intakeMotor = new SparkFlex(intakeMotorCanId, MotorType.kBrushless);
  private final RelativeEncoder feederEncoder = feeder.getEncoder();
  private final RelativeEncoder launcherEncoder = launcher.getEncoder();
  private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

  private final SparkClosedLoopController intakeController = intakeMotor.getClosedLoopController();
  private final SparkClosedLoopController feederController = feeder.getClosedLoopController();
  private final SparkClosedLoopController launcherController = launcher.getClosedLoopController();

  // private final SparkMax sweeper = new SparkMax(sweeperCanId, MotorType.kBrushed);

  // private int agitatorCount = 0;
  // private int[] agitatorDir = {1, -1};
  // private int agitatorIndex = 0;

  public SuperstructureIOSpark() {
    var feederConfig = new SparkFlexConfig();
    feederConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(feederCurrentLimit)
        .voltageCompensation(12.0)
        .inverted(true);
    feederConfig
        .encoder
        .positionConversionFactor(1 / feederMotorReduction)
        .velocityConversionFactor(1 / feederMotorReduction);
    feederConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(feederKp, 0.0, feederKd, ClosedLoopSlot.kSlot0)
        .feedForward
        .kV(feederKv);
    // feederConfig
    //     .signals
    //     .primaryEncoderPositionAlwaysOn(true)
    //     .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
    //     .primaryEncoderVelocityAlwaysOn(true)
    //     .primaryEncoderVelocityPeriodMs(20)
    //     .appliedOutputPeriodMs(20)
    //     .busVoltagePeriodMs(20)
    //     .outputCurrentPeriodMs(20);
    tryUntilOk(
        feeder,
        5,
        () ->
            feeder.configure(
                feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    var launcherConfig = new SparkFlexConfig();
    launcherConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(launcherCurrentLimit)
        .inverted(true)
        .voltageCompensation(12.0);
    launcherConfig
        .encoder
        .positionConversionFactor(1 / launcherMotorReduction)
        .velocityConversionFactor(1 / launcherMotorReduction);
    launcherConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(launcherKp, 0.0, launcherKd, ClosedLoopSlot.kSlot0)
        .feedForward
        .kV(launcherKv);
    // launcherConfig
    //     .signals
    //     .primaryEncoderPositionAlwaysOn(true)
    //     .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
    //     .primaryEncoderVelocityAlwaysOn(true)
    //     .primaryEncoderVelocityPeriodMs(20)
    //     .appliedOutputPeriodMs(20)
    //     .busVoltagePeriodMs(20)
    //     .outputCurrentPeriodMs(20);
    tryUntilOk(
        launcher,
        5,
        () ->
            launcher.configure(
                launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    var intakeConfig = new SparkFlexConfig();
    intakeConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(intakeCurrentLimit)
        .inverted(false)
        .voltageCompensation(12.0);
    intakeConfig
        .encoder
        .positionConversionFactor(1 / intakeMotorReduction)
        .velocityConversionFactor(1 / intakeMotorReduction);
    intakeConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // .pid(intakeKp, 0.0, intakeKd);
        .pid(intakeKp, 0.0, intakeKd, ClosedLoopSlot.kSlot0)
        .feedForward
        .kV(intakeKv);
    // intakeConfig
    //     .signals
    //     .primaryEncoderPositionAlwaysOn(true)
    //     .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
    //     .primaryEncoderVelocityAlwaysOn(true)
    //     .primaryEncoderVelocityPeriodMs(20)
    //     .appliedOutputPeriodMs(20)
    //     .busVoltagePeriodMs(20)
    //     .outputCurrentPeriodMs(20);
    tryUntilOk(
        intakeMotor,
        5,
        () ->
            intakeMotor.configure(
                intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // var sweeperConfig = new SparkMaxConfig();
    // sweeperConfig
    //     .idleMode(IdleMode.kBrake)
    //     .smartCurrentLimit(sweeperCurrentLimit)
    //     .inverted(false)
    //     .voltageCompensation(12.0);
    // sweeperConfig
    //     .signals
    //     .primaryEncoderPositionAlwaysOn(true)
    //     .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
    //     .primaryEncoderVelocityAlwaysOn(true)
    //     .primaryEncoderVelocityPeriodMs(20)
    //     .appliedOutputPeriodMs(20)
    //     .busVoltagePeriodMs(20)
    //     .outputCurrentPeriodMs(20);
    // tryUntilOk(
    //     sweeper,
    //     5,
    //     () ->
    //         sweeper.configure(
    //             sweeperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Timer.delay(0.1); // FIXME: idk if this is necessary

    // set up Spark Flex configuration for the left motor
    // (not sure what everything is set to but I think it works)
    // SparkFlexConfig followMotorConfig = new SparkFlexConfig();
    // followMotorConfig
    //     .idleMode(IdleMode.kBrake)
    //     .smartCurrentLimit(launcherCurrentLimit)
    //     .voltageCompensation(12.0);
    // followMotorConfig
    //     .signals
    //     .appliedOutputPeriodMs(20)
    //     .busVoltagePeriodMs(20)
    //     .outputCurrentPeriodMs(20);
    // followMotorConfig.follow(launcherCanId, true);

    // // sets the configuration of the left motor
    // tryUntilOk(
    //     launcherFollower,
    //     5,
    //     () ->
    //         launcherFollower.configure(
    //             followMotorConfig, ResetMode.kResetSafeParameters,
    // PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(SuperstructureIOInputs inputs) {
    ifOk(feeder, feederEncoder::getPosition, (value) -> inputs.feederPositionRad = value);
    ifOk(feeder, feederEncoder::getVelocity, (value) -> inputs.feederVelocityRadPerSec = value);
    ifOk(
        feeder,
        new DoubleSupplier[] {feeder::getAppliedOutput, feeder::getBusVoltage},
        (values) -> inputs.feederAppliedVolts = values[0] * values[1]);
    ifOk(feeder, feeder::getOutputCurrent, (value) -> inputs.feederCurrentAmps = value);

    ifOk(
        launcher,
        launcherEncoder::getPosition,
        (value) -> inputs.intakeLauncherPositionRad = value);
    ifOk(
        launcher,
        launcherEncoder::getVelocity,
        (value) -> inputs.intakeLauncherVelocityRadPerSec = value);
    ifOk(
        launcher,
        new DoubleSupplier[] {launcher::getAppliedOutput, launcher::getBusVoltage},
        (values) -> inputs.intakeLauncherAppliedVolts = values[0] * values[1]);
    ifOk(launcher, launcher::getOutputCurrent, (value) -> inputs.intakeLauncherCurrentAmps = value);

    Logger.recordOutput("Superstructure/Current Feeder Speed", feederEncoder.getVelocity());
    Logger.recordOutput("Superstructure/Current Intake Speed", intakeEncoder.getVelocity());
    Logger.recordOutput("Superstructure/Current Shooter Speed", launcherEncoder.getVelocity());
  }

  //   public void setDriveVelocity(double velocityRadPerSec) {
  //     manipulatorController.setSetpoint(
  //         velocityRadPerSec,
  //         ControlType.kMAXMotionVelocityControl,
  //         ClosedLoopSlot.kSlot0,
  //         0,
  //         ArbFFUnits.kVoltage);
  //   }

  @Override
  public void setFeederSpeed(double speed) {
    // feeder.set(speed);
    feederController.setSetpoint(speed, ControlType.kVelocity);
    Logger.recordOutput("Superstructure/Feeder Target Speed", speed);
  }

  @Override
  public void setIntakeSpeed(double speed) {
    // intakeMotor.set(speed);
    intakeController.setSetpoint(speed, ControlType.kVelocity);
    Logger.recordOutput("Superstructure/Intake Target Speed", speed);
  }

  public void setLauncherSpeed(double speed) {
    // launcher.set(speed);
    launcherController.setSetpoint(speed, ControlType.kVelocity);
    Logger.recordOutput("Superstructure/Shooter Target Speed", speed);
  }

  public void setSweeperSpeed(double speed) {
    // agitatorCount++;
    // sweeper.set(speed * agitatorDir[agitatorIndex]);
    // if (agitatorCount % 70 == 0) {
    //   agitatorIndex++;
    //   if (agitatorIndex > agitatorDir.length - 1) {
    //     agitatorIndex = 0;
    //   }
    // }
    // Logger.recordOutput("Superstructure/Sweeper Target Speed", speed);
  }
}
