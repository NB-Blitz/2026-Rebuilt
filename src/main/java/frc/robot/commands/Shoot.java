package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.superstructure.Superstructure;

public class Shoot extends Command {
  private Superstructure manipulator;
  private Timer timer;
  private double time;
  private boolean started = false;

  public Shoot(double time, Superstructure manipulator) {
    this.time = time;
    this.manipulator = manipulator;
    addRequirements(manipulator);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    if (!started) {
      new Trigger(() -> timer.hasElapsed(time)).whileFalse(manipulator.launch());
      started = true;
    }
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(time);
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    System.out.println("Ended immediately");
  }
}
