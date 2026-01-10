package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.manipulator.Manipulator;

public class GoToPreset extends Command {
  private Manipulator manipulator;
  private int presetIndex;

  public GoToPreset(Manipulator manipulator, int presetIndex) {
    this.manipulator = manipulator;
    this.presetIndex = presetIndex;
  }

  @Override
  public void initialize() {
    manipulator.goToPreset(presetIndex);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return manipulator.isAtTarget();
  }
}
