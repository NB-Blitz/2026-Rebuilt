// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.feederMotorReduction;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.intakeLauncherMotorReduction;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
// import edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
// import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.FuelVelocity;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.Logger;

public class SuperstructureIOSim implements SuperstructureIO {
  private DCMotorSim feederSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getCIM(1), 0.004, feederMotorReduction),
          DCMotor.getCIM(1));
  private DCMotorSim intakeLauncherSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getCIM(1), 0.004, intakeLauncherMotorReduction),
          DCMotor.getCIM(1));

  private double feederAppliedVolts = 0.0;
  private double intakeLauncherAppliedVolts = 0.0;
  private final IntakeSimulation intakeSimulation;
  private AbstractDriveTrainSimulation driveTrain;

  public SuperstructureIOSim(AbstractDriveTrainSimulation driveTrain) {
    this.driveTrain = driveTrain;
    intakeSimulation =
        IntakeSimulation.InTheFrameIntake(
            "Fuel", driveTrain, Meters.of(0.397), IntakeSimulation.IntakeSide.FRONT, 10);
  }

  @Override
  public void updateInputs(SuperstructureIOInputs inputs) {
    feederSim.setInputVoltage(feederAppliedVolts);
    feederSim.update(0.02);

    intakeLauncherSim.setInputVoltage(intakeLauncherAppliedVolts);
    intakeLauncherSim.update(0.02);

    inputs.feederPositionRad = feederSim.getAngularPositionRad();
    inputs.feederVelocityRadPerSec = feederSim.getAngularVelocityRadPerSec();
    inputs.feederAppliedVolts = feederAppliedVolts;
    inputs.feederCurrentAmps = feederSim.getCurrentDrawAmps();

    inputs.intakeLauncherPositionRad = intakeLauncherSim.getAngularPositionRad();
    inputs.intakeLauncherVelocityRadPerSec = intakeLauncherSim.getAngularVelocityRadPerSec();
    inputs.intakeLauncherAppliedVolts = intakeLauncherAppliedVolts;
    inputs.intakeLauncherCurrentAmps = intakeLauncherSim.getCurrentDrawAmps();
  }

  @Override
  public void setFeederVoltage(double volts) {
    feederAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);

    if (volts < 0) {
      intakeSimulation.startIntake();
    } else if (volts == 0) {
      intakeSimulation.stopIntake();
    } else {
      if (intakeSimulation.obtainGamePieceFromIntake()) {
        RebuiltFuelOnFly fuelOnFly =
            new RebuiltFuelOnFly(
                // Specify the position of the chassis when the note is launched
                driveTrain.getSimulatedDriveTrainPose().getTranslation(),
                // Specify the translation of the shooter from the robot center (in the shooter’s
                // reference frame)
                FuelVelocity.SHOOTER_POSITION_INV.getTranslation(),
                // Specify the field-relative speed of the chassis, adding it to the initial
                // velocity of the projectile
                driveTrain.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                // The shooter facing direction is the same as the robot’s facing direction
                driveTrain
                    .getSimulatedDriveTrainPose()
                    .getRotation()
                    .rotateBy(FuelVelocity.SHOOTER_POSITION.getRotation()),
                // Initial height of the flying note
                Distance.ofBaseUnits(FuelVelocity.ROBOT_SHOOTER_HEIGHT * 0.0254, Units.Meters),
                // The launch speed is proportional to the RPM; assumed to be 16 meters/second at
                // 6000 RPM
                LinearVelocity.ofBaseUnits(
                    // 8,
                    FuelVelocity.calcFixedLaunchVelocity(driveTrain.getSimulatedDriveTrainPose()),
                    Units.MetersPerSecond),
                // The angle at which the note is launched
                Angle.ofBaseUnits(FuelVelocity.THETA, Units.Radians));

        fuelOnFly
            // Set the target center to the Hub of the current alliance
            .withTargetPosition(
                () ->
                    FieldMirroringUtils.toCurrentAllianceTranslation(
                        new Translation3d(
                            FuelVelocity.HUB_POSITION.getTranslation().getX(),
                            FuelVelocity.HUB_POSITION.getTranslation().getY(),
                            FuelVelocity.HUB_HEIGHT)))
            // Set the tolerance
            .withTargetTolerance(
                new Translation3d(
                    FuelVelocity.HUB_DIMENTIONS.getX(), FuelVelocity.HUB_DIMENTIONS.getY(), 12))
            // Set a callback to run when the note hits the target
            .withHitTargetCallBack(() -> System.out.println("Made it in the hub"));

        fuelOnFly
            // Configure callbacks to visualize the flight trajectory of the projectile
            .withProjectileTrajectoryDisplayCallBack(
            // Callback for when the note will eventually hit the target (if configured)
            (pose3ds) ->
                Logger.recordOutput(
                    "Flywheel/NoteProjectileSuccessfulShot", pose3ds.toArray(Pose3d[]::new)),
            // Callback for when the note will eventually miss the target, or if no target is
            // configured
            (pose3ds) ->
                Logger.recordOutput(
                    "Flywheel/NoteProjectileUnsuccessfulShot", pose3ds.toArray(Pose3d[]::new)));

        fuelOnFly.enableBecomesGamePieceOnFieldAfterTouchGround();
        SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
      }
    }
  }

  @Override
  public void setIntakeLauncherVoltage(double volts) {
    if (volts < 0) {
      intakeSimulation.startIntake();
    } else if (volts == 0) {
      intakeSimulation.stopIntake();
    }
    intakeLauncherAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }
}
