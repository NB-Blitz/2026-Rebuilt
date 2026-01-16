package frc.robot.subsystems.manipulator;

public interface ElevatorInterface {
  public double noFoulPos = 0.66;

  // create an enum for preset elevator heights (ex. coral level 1, 2, 3, 4)
  public enum ElevatorPosition {
    bottom(0),
    algaeProcessor(0),
    coralStation(0.055),
    coralL1(0.29),
    frontIntake(0.3),
    coralL2(0.39),
    algaeInReefL2(0.5),
    coralL3(0.66),
    algaeInReefL3(0.7),
    coralL4(0.62),
    algaeBarge(0.75);

    public final double position;

    ElevatorPosition(double position) {
      this.position = position;
    }
  }

  // returns the state of the limit switch
  public boolean getLimit();

  // moves the elevator to a preset position specified by the Position parameter (created in the
  // enum)
  public void setPosition(ElevatorPosition position);

  public void setSpeed(double joystickInput);

  public void eStop();

  // moves the elevator a certain speed according to the double parameter
  public void move();

  public double getHeight();

  public void resetTargetHeight();

  public double getTarget();

  public double getTopLimit();
}
