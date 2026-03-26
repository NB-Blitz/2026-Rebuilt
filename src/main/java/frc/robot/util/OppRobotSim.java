package frc.robot.util;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.Logger;

public class OppRobotSim extends SubsystemBase {
  /* If an opponent robot is not on the field, it is placed in a queening position for performance. */
  public static final Pose2d[] ROBOT_QUEENING_POSITIONS =
      new Pose2d[] {
        new Pose2d(-6, 0, new Rotation2d()),
        new Pose2d(-5, 0, new Rotation2d()),
        new Pose2d(-4, 0, new Rotation2d()),
        new Pose2d(-3, 0, new Rotation2d()),
        new Pose2d(-2, 0, new Rotation2d())
      };

  private final SelfControlledSwerveDriveSimulation driveSimulation;
  private final Pose2d queeningPose;
  private final int id;

  private static final DriveTrainSimulationConfig DRIVETRAIN_CONFIG =
      DriveTrainSimulationConfig.Default();

  public static final OppRobotSim[] instances =
      new OppRobotSim[4]; // you can makes as many opponent robots as you needs

  public static void startOpponentRobotSimulations() {
    try {
      instances[0] = new OppRobotSim(0);
      instances[0].buildBehaviorChooser(
          PathPlannerPath.fromPathFile("OpponentPath1"),
          Commands.none(),
          PathPlannerPath.fromPathFile("OpponentPath1"),
          Commands.none(),
          new XboxController(2));
      instances[1] = new OppRobotSim(1);
      instances[1].buildBehaviorChooser(
          PathPlannerPath.fromPathFile("OpponentPath2"),
          Commands.none(),
          PathPlannerPath.fromPathFile("OpponentPath2"),
          Commands.none(),
          new XboxController(2));
      instances[2] = new OppRobotSim(2);
      instances[2].buildBehaviorChooser(
          PathPlannerPath.fromPathFile("OpponentPath3"),
          Commands.none(),
          PathPlannerPath.fromPathFile("OpponentPath3"),
          Commands.none(),
          new XboxController(2));
      instances[3] = new OppRobotSim(3);
      instances[3].buildBehaviorChooser(
          PathPlannerPath.fromPathFile("OpponentPath4"),
          Commands.none(),
          PathPlannerPath.fromPathFile("OpponentPath4"),
          Commands.none(),
          new XboxController(2));
    } catch (Exception e) {
      DriverStation.reportError(
          "Failed to load opponent robot simulation paths, error: "
              + e.getMessage()
              + "\n"
              + e.getStackTrace(),
          false);
      // System.out.println(
      //    "Failed to load opponent robot simulation paths, error: " + e.getMessage());
    }
  }

  public OppRobotSim(int id) {
    this.id = id;
    this.queeningPose = ROBOT_QUEENING_POSITIONS[id];
    this.driveSimulation =
        new SelfControlledSwerveDriveSimulation(
            new SwerveDriveSimulation(DRIVETRAIN_CONFIG, queeningPose));

    SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation.getDriveTrainSimulation());
  }

  // yash planner stuff
  // x = 8.270
  // y = 4.132

  // X = 0.517
  // Y = 5.957

  // PathPlanner configuration
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          55, // Robot mass in kg
          8, // Robot MOI
          new ModuleConfig(
              Units.inchesToMeters(2),
              3.5,
              1.2,
              DCMotor.getFalcon500(1).withReduction(8.14),
              60,
              1), // Swerve module config
          0.6);

  // PathPlanner PID settings
  private final PPHolonomicDriveController driveController =
      new PPHolonomicDriveController(new PIDConstants(5.0, 0.02), new PIDConstants(7.0, 0.05));

  /** Follow path command for opponent robots */
  private Command opponentRobotFollowPath(PathPlannerPath path) {
    return new FollowPathCommand(
        path, // Specify the path
        // Provide actual robot pose in simulation, bypassing odometry error
        driveSimulation::getActualPoseInSimulationWorld,
        // Provide actual robot speed in simulation, bypassing encoder measurement error
        driveSimulation::getActualSpeedsRobotRelative,
        // Chassis speeds output
        (speeds, feedforwards) ->
            driveSimulation.runChassisSpeeds(speeds, new Translation2d(), false, false),
        driveController, // Specify PID controller
        PP_CONFIG, // Specify robot configuration
        // Flip path based on alliance side
        () ->
            DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue)
                .equals(DriverStation.Alliance.Red),
        this // AIRobotInSimulation is a subsystem; this command should use it as a requirement
        );
  }

  public void buildBehaviorChooser(
      PathPlannerPath segment0,
      Command toRunAtEndOfSegment0,
      PathPlannerPath segment1,
      Command toRunAtEndOfSegment1,
      XboxController joystick) {
    SendableChooser<Command> behaviorChooser = new SendableChooser<>();
    final Supplier<Command> disable =
        () ->
            Commands.runOnce(() -> driveSimulation.setSimulationWorldPose(queeningPose), this)
                .andThen(
                    Commands.runOnce(
                        () ->
                            driveSimulation.runChassisSpeeds(
                                new ChassisSpeeds(), new Translation2d(), false, false)))
                .ignoringDisable(true);

    // Option to disable the robot
    behaviorChooser.setDefaultOption("Disable", disable.get());

    // Option to auto-cycle the robot
    behaviorChooser.addOption(
        "Auto Cycle",
        getAutoCycleCommand(segment0, toRunAtEndOfSegment0, segment1, toRunAtEndOfSegment1));

    // Option to manually control the robot with a joystick
    // behaviorChooser.addOption("Joystick Drive", joystickDrive(joystick));

    // Schedule the command when another behavior is selected
    behaviorChooser.onChange((cmd) -> CommandScheduler.getInstance().schedule(cmd));

    // Schedule the selected command when teleop starts
    RobotModeTriggers.teleop().onTrue(behaviorChooser.getSelected());

    // Disable the robot when the user robot is disabled
    RobotModeTriggers.disabled().onTrue(disable.get());

    SmartDashboard.putData(
        "OpponentRobotSimulation/Opponent Robot " + id + " Behavior", behaviorChooser);
  }

  /** Get the command to auto-cycle the robot relatively */
  private Command getAutoCycleCommand(
      PathPlannerPath segment0,
      Command toRunAtEndOfSegment0,
      PathPlannerPath segment1,
      Command toRunAtEndOfSegment1) {
    final SequentialCommandGroup cycle = new SequentialCommandGroup();
    final Pose2d startingPose =
        new Pose2d(
            segment0.getStartingDifferentialPose().getTranslation(),
            segment0.getIdealStartingState().rotation());

    cycle.addCommands(
        opponentRobotFollowPath(segment0).andThen(toRunAtEndOfSegment0).withTimeout(10));
    cycle.addCommands(
        opponentRobotFollowPath(segment1).andThen(toRunAtEndOfSegment1).withTimeout(10));

    return cycle
        .repeatedly()
        .beforeStarting(
            Commands.runOnce(
                () ->
                    driveSimulation.setSimulationWorldPose(
                        FieldMirroringUtils.toCurrentAlliancePose(startingPose))));
  }

  public static void updateSimulation() {
    for (int i = 0; i < instances.length; i++) {
      OppRobotSim robot = instances[i];
      Logger.recordOutput(
          "FieldSimulation/OppRobot" + i + "Position",
          robot.driveSimulation.getActualPoseInSimulationWorld());
    }
  }
}
