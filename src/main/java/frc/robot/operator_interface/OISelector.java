package frc.robot.operator_interface;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.List;

/**
 * Utility class for selecting the appropriate OI implementations based on the connected joysticks.
 */
public class OISelector {
  private static String[] lastJoystickNames = new String[] {null, null, null, null, null, null};
  private static final String noOperatorInterfaceWarning = "No operator controller(s) connected.";
  private static final String singleXBoxOperatorInterfaces = "XBox operator controller connected.";
  private static final String dualXBoxOperatorInterfaces =
      "Dual XBox operator controllers connected.";
  private static final String singleJoystickOperatorInterfaces = "Single Joystick connected.";
  private static final String dualJoystickOperatorInterfaces = "Dual Joysticks connected.";
  private static final String dualJoystickXBoxOperatorInterfaces =
      "Dual Joysticks and XBox controller connected.";

  private OISelector() {}

  /**
   * Returns whether the connected joysticks have changed since the last time this method was
   * called.
   */
  public static boolean didJoysticksChange() {
    boolean joysticksChanged = false;

    for (int port = 0; port < DriverStation.kJoystickPorts; port++) {
      String name = DriverStation.getJoystickName(port) + DriverStation.getJoystickType(port);
      if (!name.equals(lastJoystickNames[port])) {
        lastJoystickNames[port] = name;
        joysticksChanged = true;
      }
    }
    return joysticksChanged;
  }

  /**
   * Instantiates and returns an appropriate OperatorInterface object based on the connected
   * joysticks.
   */
  public static OperatorInterface findOperatorInterface() {
    List<Integer> joyList = new ArrayList<Integer>();
    List<Integer> xboxList = new ArrayList<Integer>();

    for (int port = 0; port < DriverStation.kJoystickPorts; port++) {
      String name = DriverStation.getJoystickName(port).toLowerCase();
      if (!name.equals("")) {
        if (name.contains("xbox") || name.contains("pad") || name.contains("keyboard")) {
          xboxList.add(port);
        } else {
          joyList.add(port);
        }
      }
    }
    if (joyList.size() > 1) {
      if (xboxList.size() > 0) {
        System.out.println(dualJoystickXBoxOperatorInterfaces);
        return new DualJoystickXboxOI(joyList.get(0), joyList.get(1), xboxList.get(0));
      } else {
        System.out.println(dualJoystickOperatorInterfaces);
        return new DualJoysticksOI(joyList.get(0), joyList.get(1));
      }
    } else if (joyList.size() > 0) {
      System.out.println(singleJoystickOperatorInterfaces);
      return new SingleHandheldOI(joyList.get(0));
    } else if (xboxList.size() > 1) {
      System.out.println(dualXBoxOperatorInterfaces);
      return new DualHandheldOI(xboxList.get(0), xboxList.get(1));
    } else if (xboxList.size() > 0) {
      System.out.println(singleXBoxOperatorInterfaces);
      return new SingleHandheldOI(xboxList.get(0));
    } else {
      DriverStation.reportWarning(noOperatorInterfaceWarning, false);
      return new OperatorInterface() {};
    }
  }
}
