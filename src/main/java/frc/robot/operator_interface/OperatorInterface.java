package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Interface for all driver and operator controls. */
public interface OperatorInterface {

  public static int DRIVER = 0;
  public static int OPERATOR = 1;

  public default double getTranslateX() {
    return 0.0;
  }

  public default double getTranslateY() {
    return 0.0;
  }

  public default double getRotate() {
    return 0.0;
  }

  public default double getRotateY() {
    return 0.0;
  }

  public default Trigger driveScalingUp() {
    return new Trigger(() -> false);
  }

  public default Trigger driveScalingDown() {
    return new Trigger(() -> false);
  }

  public default Trigger driveScalingSlow() {
    return new Trigger(() -> false);
  }

  public default double driveScalingValue() {
    return 1.0;
  }

  public default boolean isRobotRelative() {
    return false;
  }

  public default Trigger getRobotRelative() {
    return new Trigger(() -> false);
  }

  public default Trigger getResetGyroButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getXStanceButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getLeftTrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getRightTrigger() {
    return new Trigger(() -> false);
  }

  public default double getLeftTriggerValue() {
    return 0.0;
  }

  public default double getRightTriggerValue() {
    return 0.0;
  }

  public default Trigger getLeftBumper() {
    return new Trigger(() -> false);
  }

  public default Trigger getRightBumper() {
    return new Trigger(() -> false);
  }

  public default Trigger getStartButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getBackButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getXButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getYButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getAButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getBButton() {
    return new Trigger(() -> false);
  }

  public default double getLeftThumbstickX() {
    return 0.0;
  }

  public default double getLeftThumbstickY() {
    return 0.0;
  }

  public default Trigger getLeftThumbstickButton() {
    return new Trigger(() -> false);
  }

  public default double getRightThumbstickX() {
    return 0.0;
  }

  public default double getRightThumbstickY() {
    return 0.0;
  }

  public default Trigger getRightThumbstickButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getPOVUp() {
    return new Trigger(() -> false);
  }

  public default Trigger getPOVDown() {
    return new Trigger(() -> false);
  }

  public default Trigger getPOVLeft() {
    return new Trigger(() -> false);
  }

  public default Trigger getPOVRight() {
    return new Trigger(() -> false);
  }

  public default void testOI(int mode) {}

  public default boolean testResults(int mode) {
    return true;
  }
}
