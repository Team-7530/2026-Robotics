package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with a single joystick. */
public class SingleJoystickOI implements OperatorInterface {

  protected final CommandJoystick joystick;
  protected final Trigger[] joystickButtons;

  public SingleJoystickOI(int port) {
    joystick = new CommandJoystick(port);

    // buttons use 1-based indexing such that the index matches the button number; leave index 0 set
    // to null
    this.joystickButtons = new Trigger[13];

    for (int i = 1; i < joystickButtons.length; i++) {
      joystickButtons[i] = joystick.button(i);
    }
  }

  @Override
  public double getTranslateX() {
    return -joystick.getY();
  }

  @Override
  public double getTranslateY() {
    return -joystick.getX();
  }

  @Override
  public double getRotate() {
    return -joystick.getTwist();
  }

  @Override
  public double getRotateY() {
    return -joystick.getTwist();
  }

  @Override
  public Trigger driveScalingUp() {
    return new Trigger(() -> (joystick.getHID().getPOV() == 0));
  }

  @Override
  public Trigger driveScalingDown() {
    return new Trigger(() -> (joystick.getHID().getPOV() == 180));
  }

  @Override
  public Trigger driverLeftTrigger() {
    return joystickButtons[1];
  }

  @Override
  public Trigger getRobotRelative() {
    return joystickButtons[4];
  }

  @Override
  public Trigger getResetGyroButton() {
    return joystickButtons[3];
  }

  @Override
  public Trigger getXStanceButton() {
    return joystickButtons[2];
  }
}
