package frc.robot.operator_interface;

import static frc.robot.Constants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with a single joystick. */
public class SingleJoystickOI implements OperatorInterface {
  protected final CommandJoystick joystick;
  protected final Trigger[] joystickButtons;

  private double driveScaleFactor = 0.5;
  private boolean updateDriveScale = false;
  protected boolean tests[][] = new boolean[2][20];

  public SingleJoystickOI(int port) {
    joystick = new CommandJoystick(port);

    // buttons use 1-based indexing such that the index matches the button number; leave index 0 set
    // to null
    this.joystickButtons = new Trigger[13];

    for (int i = 1; i < joystickButtons.length; i++) {
      joystickButtons[i] = joystick.button(i);
    }
  }

  public void testController(CommandJoystick contrl, boolean[] test) {
    for (int testNum = 0; testNum < test.length; ++testNum) {
      if (!test[testNum]) {
        switch (testNum) {
          case 0:
            test[testNum] = MathUtil.applyDeadband(contrl.getY(), STICK_DEADBAND) > 0.0;
            break;
          case 1:
            test[testNum] = MathUtil.applyDeadband(contrl.getX(), STICK_DEADBAND) > 0.0;
            break;
          case 2:
            test[testNum] = MathUtil.applyDeadband(contrl.getTwist(), STICK_DEADBAND) > 0.0;
            break;
          case 3:
            test[testNum] = contrl.button(0).getAsBoolean();
            break;
          case 4:
            test[testNum] = contrl.button(1).getAsBoolean();
            break;
          case 5:
            test[testNum] = contrl.button(2).getAsBoolean();
            break;
          case 6:
            test[testNum] = contrl.button(3).getAsBoolean();
            break;
          case 7:
            test[testNum] = contrl.button(4).getAsBoolean();
            break;
          case 8:
            test[testNum] = contrl.button(5).getAsBoolean();
            break;
          default:
            test[testNum] = true;
            break;
        }
      }
    }
  }

  @Override
  public void testOI(int mode) {
    for (int testNum = 0; testNum < tests[mode].length; ++testNum) {
      tests[mode][testNum] = false;
    }
  }

  @Override
  public boolean testResults(int mode) {
    boolean result = true;
    if (mode == DRIVER) {
      testController(joystick, tests[mode]);

      for (boolean test : tests[mode]) {
        result = result && test;
      }
    }
    return result;
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
  public Trigger driveScalingSlow() {
    return joystickButtons[1];
  }

  @Override
  public double driveScalingValue() {
    int povVal = joystick.getHID().getPOV();

    if (updateDriveScale) {
      updateDriveScale = (povVal != -1);
    } else if (povVal == 0) {
      driveScaleFactor = MathUtil.clamp(driveScaleFactor + 0.05, 0.1, 1.0);
      System.out.println("Setting driveScaleFactor to " + driveScaleFactor);
      updateDriveScale = true;
    } else if (povVal == 180) {
      driveScaleFactor = MathUtil.clamp(driveScaleFactor - 0.05, 0.1, 1.0);
      System.out.println("Setting driveScaleFactor to " + driveScaleFactor);
      updateDriveScale = true;
    }
    return driveScaleFactor;
  }

  @Override
  public boolean isRobotRelative() {
    return joystickButtons[4].getAsBoolean();
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
