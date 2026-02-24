package frc.robot.operator_interface;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with a single Xbox controller. */
public class SingleHandheldOI implements OperatorInterface {
  private final XboxController controller;

  public SingleHandheldOI(int port) {
    controller = new XboxController(port);
  }

  @Override
  public double getTranslateX() {
    return -controller.getLeftY();
  }

  @Override
  public double getTranslateY() {
    return -controller.getLeftX();
  }

  @Override
  public double getRotate() {
    return -controller.getRightX();
  }

  @Override
  public double getRotateY() {
    return -controller.getRightY();
  }

  @Override
  public Trigger getLeftTrigger() {
    return new Trigger(() -> controller.getLeftTriggerAxis() >= 0.5);
  }

  @Override
  public Trigger getRightTrigger() {
    return new Trigger(() -> controller.getRightTriggerAxis() >= 0.5);
  }

  @Override
  public Trigger driveScalingUp() {
    return new Trigger(() -> (controller.getPOV() == 0));
  }

  @Override
  public Trigger driveScalingDown() {
    return new Trigger(() -> (controller.getPOV() == 180));
  }

  @Override
  public Trigger getRobotRelative() {
    return new Trigger(controller::getLeftBumperButton);
  }

  @Override
  public Trigger getResetGyroButton() {
    return new Trigger(controller::getLeftStickButton);
  }

  @Override
  public Trigger getXStanceButton() {
    return new Trigger(controller::getRightBumperButton);
  }

  @Override
  public Trigger getAButton() {
    return new Trigger(controller::getAButton);
  }

  @Override
  public Trigger getBButton() {
    return new Trigger(controller::getBButton);
  }

  @Override
  public Trigger getXButton() {
    return new Trigger(controller::getXButton);
  }

  @Override
  public Trigger getYButton() {
    return new Trigger(controller::getYButton);
  }

  @Override
  public Trigger getStartButton() {
    return new Trigger(controller::getStartButton);
  }

  @Override
  public Trigger getBackButton() {
    return new Trigger(controller::getBackButton);
  }

  @Override
  public double getLeftTriggerValue() {
    return controller.getLeftTriggerAxis();
  }

  @Override
  public double getRightTriggerValue() {
    return controller.getRightTriggerAxis();
  }

  @Override
  public Trigger getLeftBumper() {
    return new Trigger(controller::getLeftBumperButton);
  }

  @Override
  public Trigger getRightBumper() {
    return new Trigger(controller::getRightBumperButton);
  }

  @Override
  public double getLeftThumbstickX() {
    return controller.getLeftX();
  }

  @Override
  public double getLeftThumbstickY() {
    return controller.getLeftY();
  }

  @Override
  public Trigger getLeftThumbstickButton() {
    return new Trigger(() -> controller.getLeftStickButton());
  }

  @Override
  public double getRightThumbstickX() {
    return controller.getRightX();
  }

  @Override
  public double getRightThumbstickY() {
    return controller.getRightY();
  }

  @Override
  public Trigger getRightThumbstickButton() {
    return new Trigger(() -> controller.getRightStickButton());
  }

  @Override
  public Trigger getPOVUp() {
    return new Trigger(
        () -> (controller.getPOV() == 0) || (controller.getPOV() == 45) || (controller.getPOV() == 315));
  }

  @Override
  public Trigger getPOVDown() {
    return new Trigger(
        () ->
            (controller.getPOV() == 180) || (controller.getPOV() == 225) || (controller.getPOV() == 135));
  }

  @Override
  public Trigger getPOVLeft() {
    return new Trigger(
        () ->
            (controller.getPOV() == 270) || (controller.getPOV() == 315) || (controller.getPOV() == 225));
  }

  @Override
  public Trigger getPOVRight() {
    return new Trigger(
        () -> (controller.getPOV() == 90) || (controller.getPOV() == 135) || (controller.getPOV() == 45));
  }
}
