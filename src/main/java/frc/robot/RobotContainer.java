// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ReefAlign;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkFlex;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.drive.constants.DriveConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.LEDStrip;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;

  // Constant to switch between the practice SDS base and the competition Flex base
  private final boolean useSecondController = true;
  private final boolean useXboxControllerDrive = true;

  private final boolean useManipulator = false;
  private final Superstructure manipulator;

  // Controllers
  private final CommandJoystick joystick;
  private final CommandXboxController xBoxController;
  private final CommandXboxController driveXboxController;
  private final CommandGenericHID driverStation = new CommandGenericHID(2);

  private final LEDStrip ledStrip = new LEDStrip(9, 58);
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (useSecondController) {
      xBoxController = new CommandXboxController(1);
    } else {
      xBoxController = null;
    }

    if(useManipulator){
      manipulator = new Superstructure(new SuperstructureIOSpark());
    } else {
      manipulator = null;
    }

    if (useXboxControllerDrive) {
      driveXboxController = new CommandXboxController(0);
      joystick = null;
    } else {
      joystick = new CommandJoystick(0);
      driveXboxController = null;
    }
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        if (DriveConstants.compRobot) {
          drive =
              new Drive(
                  new GyroIONavX(),
                  new ModuleIOSparkFlex(0),
                  new ModuleIOSparkFlex(1),
                  new ModuleIOSparkFlex(2),
                  new ModuleIOSparkFlex(3));
        } else {
          drive =
              new Drive(
                  new GyroIONavX(),
                  new ModuleIOSparkMax(0),
                  new ModuleIOSparkMax(1),
                  new ModuleIOSparkMax(2),
                  new ModuleIOSparkMax(3));
        }
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                new VisionIOPhotonVision(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        // Replayed robot, disable IO implementations
        // (Use same number of dummy implementations as the real robot)
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    SmartDashboard.putBoolean("Demo Mode", false);
    Trigger demoModeToggle = new Trigger(() -> SmartDashboard.getBoolean("Demo Mode", false));
    demoModeToggle.onTrue(Commands.runOnce(() -> drive.demoMode = true, drive));
    demoModeToggle.onFalse(Commands.runOnce(() -> drive.demoMode = false, drive));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default Commands, normal field-relative drive
    if (useXboxControllerDrive) {
      drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              drive,
              () -> -1 * driveXboxController.getLeftY(),
              () -> -1 * driveXboxController.getLeftX(),
              () -> -1 * driveXboxController.getRightX(),
              () -> 0.1)); // 0.5 * (1 + -driveXboxController.getRightTriggerAxis())));
      // Lock to 0° when A button is held
      // joystick
      //     .button(11)
      //     .whileTrue(
      //         DriveCommands.joystickDriveAtAngle(
      //             drive, () -> -joystick.getY(), () -> -joystick.getX(), () -> new
      // Rotation2d()));

      // Switch to X pattern when X button is pressed
      driveXboxController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

      // Reset gyro to 0° when B button is pressed
      driveXboxController
          .povDown()
          .onTrue(Commands.runOnce(() -> drive.resetGyro(), drive).ignoringDisable(true));

      // Align to reef positions
      driveXboxController
          .rightBumper()
          .whileTrue(
              new ReefAlign(drive, vision, () -> vision.getReefTags(0), Constants.rightReef[1]));
      driveXboxController
          .leftBumper()
          .whileTrue(
              new ReefAlign(drive, vision, () -> vision.getReefTags(0), Constants.leftReef[1]));

    } else {
      drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              drive,
              () -> -1 * joystick.getY(),
              () -> -1 * joystick.getX(),
              () -> -1 * joystick.getZ(),
              () -> 0.5 * (1 + -joystick.getRawAxis(3))));
      // Lock to 0° when A button is held
      // joystick
      //     .button(11)
      //     .whileTrue(
      //         DriveCommands.joystickDriveAtAngle(
      //             drive, () -> -joystick.getY(), () -> -joystick.getX(), () -> new
      // Rotation2d()));

      // Switch to X pattern when X button is pressed
      joystick.button(4).onTrue(Commands.runOnce(drive::stopWithX, drive));

      // Reset gyro to 0° when B button is pressed
      joystick
          .button(7)
          .onTrue(Commands.runOnce(() -> drive.resetGyro(), drive).ignoringDisable(true));

      // Align to reef positions
      joystick
          .button(6)
          .whileTrue(
              new ReefAlign(drive, vision, () -> vision.getReefTags(0), Constants.rightReef[1]));
      joystick
          .button(5)
          .whileTrue(
              new ReefAlign(drive, vision, () -> vision.getReefTags(0), Constants.leftReef[1]));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
