package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.manipulator.Manipulator;

public class ExpelCoral extends Command {
  private Manipulator manipulator;

  public ExpelCoral(Manipulator manipulator) {
    this.manipulator = manipulator;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    manipulator.expelCoralIntakeAlgaeAuto();
  }

  @Override
  public boolean isFinished() {
    return !manipulator.isHoldingCoral();
  }

  @Override
  public void end(boolean interrupted) {
    manipulator.stopHand();
  }
}
