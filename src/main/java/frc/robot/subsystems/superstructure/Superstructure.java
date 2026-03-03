// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.superstructure;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private final SuperstructureIO io;
  private final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged();
  private double launchingSpeed;

  public Superstructure(SuperstructureIO io) {
    this.io = io;
    SmartDashboard.putNumber("Shooter RPM", SuperstructureConstants.launchingLauncherSpeed);
  }

  @Override
  public void periodic() {
    launchingSpeed =
        SmartDashboard.getNumber("Shooter RPM", SuperstructureConstants.launchingLauncherSpeed);
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
        },
        () -> {
          io.setFeederSpeed(0.0);
          io.setIntakeSpeed(0.0);
        });
  }

  /** Set the rollers to the values for launching. Spins up before feeding fuel. */
  public Command launch() {
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
                }))
        .finallyDo(
            () -> {
              io.setFeederSpeed(0.0);
              io.setLauncherSpeed(0.0);
              io.setIntakeSpeed(0);
            });
  }
}
