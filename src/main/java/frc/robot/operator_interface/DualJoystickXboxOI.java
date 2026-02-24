package frc.robot.operator_interface;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with two Xbox controllers. */
public class DualJoystickXboxOI extends DualJoysticksOI {
  public OperatorInterface testOI = new OperatorInterface() {};
  private final XboxController operator;

  public DualJoystickXboxOI(int translatePort, int rotatePort, int xboxPort) {
    super(translatePort, rotatePort);
    operator = new XboxController(xboxPort);
  }

  public DualJoystickXboxOI(int translatePort, int rotatePort, int xboxPort, int testPort) {
    this(translatePort, rotatePort, xboxPort);
    testOI = new SingleHandheldOI(testPort);
  }

  @Override
  public Trigger getLeftTrigger() {
    return new Trigger(() -> operator.getLeftTriggerAxis() >= 0.5);
  }

  @Override
  public Trigger getRightTrigger() {
    return new Trigger(() -> operator.getRightTriggerAxis() >= 0.5);
  }

  @Override
  public double getLeftTriggerValue() {
    return operator.getLeftTriggerAxis();
  }

  @Override
  public double getRightTriggerValue() {
    return operator.getRightTriggerAxis();
  }

  @Override
  public Trigger getLeftBumper() {
    return new Trigger(operator::getLeftBumperButton);
  }

  @Override
  public Trigger getRightBumper() {
    return new Trigger(operator::getRightBumperButton);
  }

  @Override
  public Trigger getAButton() {
    return new Trigger(operator::getAButton);
  }

  @Override
  public Trigger getBButton() {
    return new Trigger(operator::getBButton);
  }

  @Override
  public Trigger getXButton() {
    return new Trigger(operator::getXButton);
  }

  @Override
  public Trigger getYButton() {
    return new Trigger(operator::getYButton);
  }

  @Override
  public Trigger getStartButton() {
    return new Trigger(operator::getStartButton);
  }

  @Override
  public Trigger getBackButton() {
    return new Trigger(operator::getBackButton);
  }

  @Override
  public double getLeftThumbstickX() {
    return operator.getLeftX();
  }

  @Override
  public double getLeftThumbstickY() {
    return operator.getLeftY();
  }

  @Override
  public Trigger getLeftThumbstickButton() {
    return new Trigger(() -> operator.getLeftStickButton());
  }

  @Override
  public double getRightThumbstickX() {
    return operator.getRightX();
  }

  @Override
  public double getRightThumbstickY() {
    return operator.getRightY();
  }

  @Override
  public Trigger getRightThumbstickButton() {
    return new Trigger(() -> operator.getLeftStickButton());
  }

  @Override
  public Trigger getPOVUp() {
    return new Trigger(
        () -> (operator.getPOV() == 0) || (operator.getPOV() == 45) || (operator.getPOV() == 315));
  }

  @Override
  public Trigger getPOVDown() {
    return new Trigger(
        () ->
            (operator.getPOV() == 180) || (operator.getPOV() == 225) || (operator.getPOV() == 135));
  }

  @Override
  public Trigger getPOVLeft() {
    return new Trigger(
        () ->
            (operator.getPOV() == 270) || (operator.getPOV() == 315) || (operator.getPOV() == 225));
  }

  @Override
  public Trigger getPOVRight() {
    return new Trigger(
        () -> (operator.getPOV() == 90) || (operator.getPOV() == 135) || (operator.getPOV() == 45));
  }

  @Override
  public OperatorInterface getTestOI() {
    return testOI;
  }
}
