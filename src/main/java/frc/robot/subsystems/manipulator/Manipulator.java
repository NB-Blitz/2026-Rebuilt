package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.common.Joint;
import frc.robot.subsystems.manipulator.ElevatorInterface.ElevatorPosition;
import frc.robot.subsystems.manipulator.Shoulder.ShoulderAngle;
import frc.robot.subsystems.manipulator.Wrist.WristAngle;
import frc.robot.util.LEDStrip;

public class Manipulator extends SubsystemBase {
  // Set to "Blank" versions to disable each component
  private final ElevatorInterface elevator = new Elevator();
  private final Joint shoulder = new Shoulder();
  private final Joint wrist = new Wrist();
  private final HandInterface hand = new Hand();
  private final LEDStrip ledStrip;

  // private final double elevatorHeightTolerance = 0.025; // meters
  private final double shoulderNoFoulTolerance = 28.0; // degrees
  private final double elevatorTargetTolerance = 0.02;
  private final double jointTargetTolerance = 1.0;
  private int cachedWristIndex = 0;

  // private final String[] presetNames = {
  //   "Bottom",
  //   // "Processor",
  //   "Intake Ramp",
  //   "Coral L1",
  //   // "Front Intake",
  //   "Coral L2",
  //   // "Algae L2",
  //   "Coral L3",
  //   // "Algae L3",
  //   "Coral L4",
  //   "Barge"
  // };

  private final ElevatorPosition[] elevatorPositions = {
    ElevatorPosition.bottom,
    // ElevatorPosition.algaeProcessor,
    ElevatorPosition.coralStation,
    ElevatorPosition.coralL1,
    // ElevatorPosition.frontIntake,
    ElevatorPosition.coralL2,
    // ElevatorPosition.algaeInReefL2,
    ElevatorPosition.coralL3,
    // ElevatorPosition.algaeInReefL3,
    ElevatorPosition.coralL4,
    ElevatorPosition.algaeBarge
  };

  private final ShoulderAngle[] shoulderAngles = {
    ShoulderAngle.bottom,
    // ShoulderAngle.algaeProcessor,
    ShoulderAngle.coralStation,
    ShoulderAngle.coralL1,
    // ShoulderAngle.frontIntake,
    ShoulderAngle.coralL2,
    // ShoulderAngle.algaeInReefL2,
    ShoulderAngle.coralL3,
    // ShoulderAngle.algaeInReefL3,
    ShoulderAngle.coralL4,
    ShoulderAngle.algaeBarge
  };

  private final WristAngle[] wristAngles = {
    WristAngle.bottom,
    // WristAngle.algaeProcessor,
    WristAngle.coralStation,
    WristAngle.coralL1,
    // WristAngle.frontIntake,
    WristAngle.coralL2,
    // WristAngle.algaeInReefL2,
    WristAngle.coralL3,
    // WristAngle.algaeInReefL3,
    WristAngle.coralL4,
    WristAngle.algaeBarge
  };

  // 0-11
  private int levelIndex = 0;
  private boolean positionCommand = false;
  private boolean joystickCommand = true;
  private boolean isIntakingCoral = false;
  private boolean isExpellingCoral = false;
  private boolean forwardLimit = false;

  public Manipulator(LEDStrip ledStrip) {
    this.ledStrip = ledStrip;
  }

  public void periodic() {
    /**
     * Any automatic behavior we want An example would be if we have a note and are in the top
     * position we could start the shooting motors.
     */
    elevator.move();
    shoulder.updateJoint();
    wrist.updateJoint();

    if (!forwardLimit && !hand.holdingCoral()) {
      forwardLimit = true;
    }

    if (joystickCommand) {
      double smallestDiff = Integer.MAX_VALUE;
      double elevatorHeight = elevator.getHeight();
      double shoulderPos = shoulder.getPosition();
      double wristPos = wrist.getPosition();
      for (int i = 0; i < elevatorPositions.length; i++) {
        double elevatorDiff = Math.abs(elevatorPositions[i].position - elevatorHeight);
        double shoulderDiff = Math.abs(shoulderAngles[i].angle - shoulderPos);
        double wristDiff = Math.abs(wristAngles[i].angle - wristPos);
        double totalDiff = elevatorDiff + shoulderDiff + wristDiff;
        if (totalDiff < smallestDiff) {
          levelIndex = i;
          smallestDiff = totalDiff;
        }
      }
    }

    SmartDashboard.putBoolean("Home", levelIndex == 0);
    SmartDashboard.putBoolean("Intake", levelIndex == 1);
    SmartDashboard.putBoolean("L1", levelIndex == 2);
    SmartDashboard.putBoolean("L2", levelIndex == 3);
    SmartDashboard.putBoolean("L3", levelIndex == 4);
    SmartDashboard.putBoolean("L4", levelIndex == 5);
    SmartDashboard.putBoolean("Barge", levelIndex == 6);

    SmartDashboard.putBoolean("Elevator Switch", elevator.getLimit());
    SmartDashboard.putBoolean("Coral Sensor", hand.holdingCoral());
    // SmartDashboard.putBoolean("Preset Control", positionCommand);
    SmartDashboard.putBoolean("Manual Control", joystickCommand);

    double ledRatio = elevator.getHeight() / elevator.getTopLimit();
    ledStrip.updateLEDs(levelIndex, hand.holdingCoral(), ledRatio);
  }

  public void incrementLevel() {
    if (levelIndex < elevatorPositions.length - 1) levelIndex++;
    joystickCommand = false;
    positionCommand = true;
  }

  public void decrementLevel() {
    if (levelIndex > 0) levelIndex--;
    joystickCommand = false;
    positionCommand = true;
  }

  public void intakeCoralExpelAlgae() {
    isIntakingCoral = true;
    if (isExpellingCoral) {
      hand.stopMotors();
    } else {
      if (hand.holdingCoral() && (shoulder.getPosition() < 130 || wrist.getPosition() < 100)) {
        hand.stopMotors();
      } else {
        hand.intakeCoral();
      }
    }
  }

  public void expelCoralIntakeAlgae() {
    isExpellingCoral = true;
    if (isIntakingCoral) {
      hand.stopMotors();
    } else {
      if (hand.holdingCoral()
          && (forwardLimit || (shoulder.getPosition() > 130 && wrist.getPosition() > 100))) {
        hand.stopMotors();
      } else {
        hand.expelCoral();
      }
    }
  }

  public void expelCoralIntakeAlgaeAuto() {
    isExpellingCoral = true;
    if (isIntakingCoral) {
      hand.stopMotors();
    } else {
      if (hand.holdingCoral()
          && (forwardLimit || (shoulder.getPosition() > 130 && wrist.getPosition() > 100))) {
        hand.stopMotors();
      } else {
        hand.expelCoralAuto();
      }
    }
  }

  public void stopIntakeCoral() {
    isIntakingCoral = false;
    if (!isExpellingCoral) {
      hand.stopMotors();
    }
  }

  public void stopExpelCoral() {
    isExpellingCoral = false;
    if (!isIntakingCoral) {
      hand.stopMotors();
    }
    forwardLimit = false;
  }

  public void emergencyStop() {
    elevator.eStop();
    shoulder.eStop();
    wrist.eStop();
    hand.stopMotors();
  }

  public void runManipulator(double triggers, double leftJoy, double rightJoy) {
    // if we recieve a joystick input stop both auto positioning
    if (triggers == 0.0 && leftJoy == 0.0 && rightJoy == 0.0 && positionCommand) {
      elevator.setPosition(elevatorPositions[levelIndex]);
      shoulder.setJointAngle(shoulderAngles[levelIndex].ordinal());
      wrist.setJointAngle(wristAngles[levelIndex].ordinal());
      cachedWristIndex = levelIndex;
    } else if (joystickCommand || triggers != 0.0 || leftJoy != 0.0 || rightJoy != 0.0) {
      elevator.setSpeed(triggers);
      shoulder.setJointSpeed(leftJoy);
      wrist.setJointSpeed(rightJoy);
      joystickCommand = true;
    }
    positionCommand = false;

    if (shoulder.getPosition() > shoulder.noFoulPos - shoulderNoFoulTolerance
        && shoulder.getPosition() < shoulder.noFoulPos + shoulderNoFoulTolerance) {
      double angleDiff = shoulder.noFoulPos - shoulder.getPosition();
      wrist.setJointAngleRaw(wrist.noFoulPos + angleDiff);
    } else if (!joystickCommand) {
      wrist.setJointAngle(wristAngles[cachedWristIndex].ordinal());
    }
  }

  public void resetTargets() {
    elevator.resetTargetHeight();
    shoulder.resetTargetAngle();
    wrist.resetTargetAngle();
  }

  // Autonomous utility functions
  public void goToPreset(int presetIndex) {
    levelIndex = presetIndex;
    joystickCommand = false;
    positionCommand = true;
    elevator.setPosition(elevatorPositions[levelIndex]);
    shoulder.setJointAngle(shoulderAngles[levelIndex].ordinal());
    wrist.setJointAngle(wristAngles[levelIndex].ordinal());
    cachedWristIndex = levelIndex;
  }

  public boolean isAtTarget() {
    boolean elevatorAtTarget =
        elevator.getHeight() < elevator.getTarget() + elevatorTargetTolerance
            && elevator.getHeight() > elevator.getTarget() - elevatorTargetTolerance;
    boolean shoulderAtTarget =
        shoulder.getPosition() < shoulder.getTarget() + jointTargetTolerance
            && shoulder.getPosition() > shoulder.getTarget() - jointTargetTolerance;
    boolean wristAtTarget =
        wrist.getPosition() < wrist.getTarget() + jointTargetTolerance
            && wrist.getPosition() > wrist.getTarget() - jointTargetTolerance;
    return elevatorAtTarget && shoulderAtTarget && wristAtTarget;
  }

  public boolean isHoldingCoral() {
    return hand.holdingCoral();
  }

  public void stopHand() {
    isExpellingCoral = false;
    isIntakingCoral = false;
    hand.stopMotors();
  }
}
