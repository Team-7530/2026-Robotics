package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with two Xbox controllers. */
public class DualJoysticksOI implements OperatorInterface {
  private final CommandJoystick translateJoystick;
  private final CommandJoystick rotateJoystick;
  private final Trigger[] translateJoystickButtons;
  private final Trigger[] rotateJoystickButtons;

  public DualJoysticksOI(int translatePort, int rotatePort) {
    translateJoystick = new CommandJoystick(translatePort);
    rotateJoystick = new CommandJoystick(rotatePort);

    // buttons use 1-based indexing such that the index matches the button number; leave index 0 set
    // to null
    this.translateJoystickButtons = new Trigger[13];
    this.rotateJoystickButtons = new Trigger[13];

    for (int i = 1; i < translateJoystickButtons.length; i++) {
      translateJoystickButtons[i] = translateJoystick.button(i);
      rotateJoystickButtons[i] = rotateJoystick.button(i);
    }
  }

  @Override
  public double getTranslateX() {
    return -translateJoystick.getY();
  }

  @Override
  public double getTranslateY() {
    return -translateJoystick.getX();
  }

  @Override
  public double getRotate() {
    return -rotateJoystick.getX();
  }

  @Override
  public double getRotateY() {
    return -rotateJoystick.getY();
  }

  @Override
  public Trigger driveScalingUp() {
    return new Trigger(() -> (translateJoystick.getHID().getPOV() == 0));
  }

  @Override
  public Trigger driveScalingDown() {
    return new Trigger(() -> (translateJoystick.getHID().getPOV() == 180));
  }

  @Override
  public Trigger driverLeftTrigger() {
    return translateJoystickButtons[1];
  }

  @Override
  public Trigger driverRightTrigger() {
    return rotateJoystickButtons[1];
  }

  @Override
  public Trigger getRobotRelative() {
    return rotateJoystickButtons[4];
  }

  @Override
  public Trigger getResetGyroButton() {
    return rotateJoystickButtons[3];
  }

  @Override
  public Trigger getXStanceButton() {
    return rotateJoystickButtons[2];
  }
}
