package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.manipulator.Manipulator;
import java.util.function.DoubleSupplier;

public class ManipulatorCommands {

  public ManipulatorCommands() {}

  public static Command joystickManipulator(
      Manipulator manipulator,
      DoubleSupplier leftTriggerSupplier,
      DoubleSupplier rightTriggerSupplier,
      DoubleSupplier leftYSupplier,
      DoubleSupplier rightYSupplier) {
    return Commands.run(
        () -> {
          double leftT = MathUtil.applyDeadband(leftTriggerSupplier.getAsDouble(), 0.05);
          double rightT = MathUtil.applyDeadband(rightTriggerSupplier.getAsDouble(), 0.05);
          double leftY = MathUtil.applyDeadband(leftYSupplier.getAsDouble(), 0.1);
          double rightY = MathUtil.applyDeadband(rightYSupplier.getAsDouble(), 0.1);
          double triggers = 0.0;

          if (leftT == 0.0 && rightT != 0.0) {
            triggers = rightT;
          } else if (leftT != 0.0 && rightT == 0.0) {
            triggers = leftT;
          }
          manipulator.runManipulator(triggers, leftY, rightY);
        },
        manipulator);
  }
}
