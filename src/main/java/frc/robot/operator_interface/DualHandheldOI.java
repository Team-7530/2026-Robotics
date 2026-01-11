package frc.robot.operator_interface;

// import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with a dual Xbox controllers. */
public class DualHandheldOI extends SingleHandheldOI {
  private final XboxController operator;

  public DualHandheldOI(int port0, int port1) {
    super(port0);
    operator = new XboxController(port1);
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
  public boolean testResults(int mode) {
    boolean result = true;
    if (mode == DRIVER) {
      result = super.testResults(mode);
    } else if (mode == OPERATOR) {
      testController(operator, tests[mode]);

      for (boolean test : tests[mode]) {
        result = result && test;
      }
    }
    return result;
  }
}
