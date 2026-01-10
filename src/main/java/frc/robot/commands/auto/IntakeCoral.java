package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.manipulator.Manipulator;

public class IntakeCoral extends Command {
  private Manipulator manipulator;

  public IntakeCoral(Manipulator manipulator) {
    this.manipulator = manipulator;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    manipulator.intakeCoralExpelAlgae();
  }

  @Override
  public boolean isFinished() {
    return manipulator.isHoldingCoral();
  }
}
