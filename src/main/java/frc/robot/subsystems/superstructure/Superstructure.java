// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.superstructure;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FuelVelocity;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private final SuperstructureIO io;
  private final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged();
  private double launchingSpeed;
  public boolean useCalcVelocity = true;
  private Supplier<Pose2d> drivePose;

  public Superstructure(SuperstructureIO io, Supplier<Pose2d> drivePose) {
    this.io = io;
    this.drivePose = drivePose;
    // SmartDashboard.putNumber("Shooter RPM", SuperstructureConstants.launchingLauncherSpeed);
  }

  @Override
  public void periodic() {
    // launchingSpeed =
    //     SmartDashboard.getNumber("Shooter RPM", SuperstructureConstants.launchingLauncherSpeed);
    io.updateInputs(inputs);
    Logger.processInputs("Superstructure", inputs);
  }

  /** Set the rollers to the values for intaking. */
  public Command intake() {
    return runEnd(
        () -> {
          io.setFeederSpeed(intakingFeederSpeed);
          io.setIntakeSpeed(intakingIntakeSpeed);
        },
        () -> {
          io.setFeederSpeed(0.0);
          io.setIntakeSpeed(0.0);
        });
  }

  /** Set the rollers to the values for ejecting fuel out the intake. */
  public Command eject() {
    return runEnd(
        () -> {
          io.setFeederSpeed(-intakingFeederSpeed);
          io.setIntakeSpeed(-intakingIntakeSpeed);
          // io.setSweeperSpeed(sweeperSpeed);
        },
        () -> {
          io.setFeederSpeed(0.0);
          io.setIntakeSpeed(0.0);
          // io.setSweeperSpeed(0.0);
        });
  }

  /** Set the rollers to the values for launching. Spins up before feeding fuel. */
  public Command launch() {

    if (useCalcVelocity == true) { // use calculated velocity

      return run(() -> {
            double startSpeed = FuelVelocity.calcFixedLaunchVelocity(drivePose.get());

            // do some math to convert to rpm
            // startSpeed = startSpeed * 2 * 60.0 / FuelVelocity.WHEEL_CIRCUMFERENCE;
            startSpeed = calcVelToRealRPM(startSpeed);

            // clamp it
            startSpeed = MathUtil.clamp(startSpeed, FuelVelocity.MIN_RPM, FuelVelocity.MAX_RPM);

            final double START_SPEED = startSpeed;
            io.setLauncherSpeed(START_SPEED);
          })
          .withTimeout(spinUpSeconds)
          .andThen(
              run(
                  () -> {
                    double speed = FuelVelocity.calcFixedLaunchVelocity(drivePose.get());

                    // do some math to convert to rpm
                    // speed = speed * 2 * 60.0 / FuelVelocity.WHEEL_CIRCUMFERENCE;
                    speed = calcVelToRealRPM(speed);

                    // clamp it
                    speed = MathUtil.clamp(speed, FuelVelocity.MIN_RPM, FuelVelocity.MAX_RPM);

                    final double SPEED = speed;

                    io.setFeederSpeed(launchingFeederSpeed);
                    io.setLauncherSpeed(SPEED);
                    io.setIntakeSpeed(launchingIntakeSpeed);
                    // io.setSweeperSpeed(sweeperSpeed);
                  }))
          .finallyDo(
              () -> {
                io.setFeederSpeed(0.0);
                io.setLauncherSpeed(0.0);
                io.setIntakeSpeed(0);
                // io.setSweeperSpeed(0.0);
              });

    } else { // run at a constant speed

      return run(() -> {
            io.setLauncherSpeed(launchingSpeed);
          })
          .withTimeout(spinUpSeconds)
          .andThen(
              run(
                  () -> {
                    io.setFeederSpeed(launchingFeederSpeed);
                    io.setLauncherSpeed(launchingSpeed);
                    io.setIntakeSpeed(launchingIntakeSpeed);
                    // io.setSweeperSpeed(sweeperSpeed);
                  }))
          .finallyDo(
              () -> {
                io.setFeederSpeed(0.0);
                io.setLauncherSpeed(0.0);
                io.setIntakeSpeed(0);
                // io.setSweeperSpeed(0.0);
              });
    }
  }

  public static double calcVelToRealRPM(double calculatedVel) {
    return calculatedVel * 600.0 - 285; // + 62.9;
  }
}
