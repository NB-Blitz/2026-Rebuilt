package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.superstructure.Superstructure;

public class Intake extends Command {
  private Superstructure manipulator;
  private Timer timer;
  private double time;
  private boolean finished = false;

  public Intake(double time, Superstructure manipulator) {
    this.time = time;
    this.manipulator = manipulator;
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    new Trigger(() -> timer.get() < time)
        .whileTrue(manipulator.intake())
        .onFalse(new InstantCommand(() -> finished = true));
  }

  @Override
  public boolean isFinished() {
    if (finished) timer.stop();
    return finished;
  }
}
