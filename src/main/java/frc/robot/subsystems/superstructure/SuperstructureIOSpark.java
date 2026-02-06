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
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import java.util.function.DoubleSupplier;

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
  // private final SparkClosedLoopController manipulatorController;

  public SuperstructureIOSpark() {
    var feederConfig = new SparkFlexConfig();
    feederConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(feederCurrentLimit)
        .voltageCompensation(12.0);
    feederConfig
        .encoder
        .positionConversionFactor(
            2.0 * Math.PI / feederMotorReduction) // Rotor Rotations -> Roller Radians
        .velocityConversionFactor((2.0 * Math.PI) / 60.0 / feederMotorReduction)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    // feederConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.1, 0, 0);
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
        .positionConversionFactor(
            2.0 * Math.PI / launcherMotorReduction) // Rotor Rotations -> Roller Radians
        .velocityConversionFactor((2.0 * Math.PI) / 60.0 / launcherMotorReduction)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    tryUntilOk(
        launcher,
        5,
        () ->
            launcher.configure(
                launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // manipulatorController = launcher.getClosedLoopController();

    var intakeConfig = new SparkFlexConfig();
    intakeConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(intakeCurrentLimit)
        .inverted(true)
        .voltageCompensation(12.0);
    intakeConfig
        .encoder
        .positionConversionFactor(
            2.0 * Math.PI / intakeMotorReduction) // Rotor Rotations -> Roller Radians
        .velocityConversionFactor((2.0 * Math.PI) / 60.0 / intakeMotorReduction)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    tryUntilOk(
        intakeMotor,
        5,
        () ->
            intakeMotor.configure(
                intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

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
  }

  @Override
  public void setFeederSpeed(double speed) {
    feeder.set(speed);
  }

  @Override
  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void setLauncherSpeed(double speed) {
    launcher.set(speed);
  }
}
