package frc.robot.subsystems.common;

public class JointBlank extends Joint {

  public JointBlank() {
    super();
  }

  public void resetEncoder(double angle) {}

  public double getPosition() {
    return 0;
  }

  public void setJointSpeed(double joystickInput) {}

  public void setJointAngle(int enumIndex) {}

  public void eStop() {}

  public void resetTargetAngle() {}

  public double getTarget() {
    return 0;
  }

  public void updateJoint() {}
}
