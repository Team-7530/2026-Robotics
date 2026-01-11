package frc.lib.util;

import edu.wpi.first.math.util.Units;

public class SwerveDriveConstants {
  public final double wheelDiameter;
  public final double wheelCircumference;
  public final double angleGearRatio;
  public final double driveGearRatio;
  public final boolean driveMotorInvert;
  public final boolean angleMotorInvert;
  public final boolean canCoderInvert;

  public SwerveDriveConstants(
      double wheelDiameter,
      double angleGearRatio,
      double driveGearRatio,
      boolean driveMotorInvert,
      boolean angleMotorInvert,
      boolean canCoderInvert) {
    this.wheelDiameter = wheelDiameter;
    this.wheelCircumference = wheelDiameter * Math.PI;
    this.angleGearRatio = angleGearRatio;
    this.driveGearRatio = driveGearRatio;
    this.driveMotorInvert = driveMotorInvert;
    this.angleMotorInvert = angleMotorInvert;
    this.canCoderInvert = canCoderInvert;
  }

  /** Swerve Drive Specialties - MK3 Module */
  public static SwerveDriveConstants SDSMK3(double driveGearRatio) {
    double wheelDiameter = Units.inchesToMeters(4.0);
    double angleGearRatio = 12.8;
    /** 12.8 : 1 */
    boolean driveMotorInvert = false;
    boolean angleMotorInvert = false;
    boolean canCoderInvert = false;
    return new SwerveDriveConstants(
        wheelDiameter,
        angleGearRatio,
        driveGearRatio,
        driveMotorInvert,
        angleMotorInvert,
        canCoderInvert);
  }

  /** SDS MK3 - 8.16 : 1 */
  public static SwerveDriveConstants SDSMK3_Standard() {
    return SDSMK3(8.16);
  }

  /** SDS MK3 - 6.86 : 1 */
  public static SwerveDriveConstants SDSMK3_Fast() {
    return SDSMK3(6.86);
  }

  /** Swerve Drive Specialties - MK4 Module */
  public static SwerveDriveConstants SDSMK4(double driveGearRatio) {
    double wheelDiameter = Units.inchesToMeters(4.0);
    double angleGearRatio = 12.8;
    /** 12.8 : 1 */
    boolean driveMotorInvert = false;
    boolean angleMotorInvert = false;
    boolean canCoderInvert = false;
    return new SwerveDriveConstants(
        wheelDiameter,
        angleGearRatio,
        driveGearRatio,
        driveMotorInvert,
        angleMotorInvert,
        canCoderInvert);
  }

  /** SDS MK4 - 8.14 : 1 */
  public static SwerveDriveConstants SDSMK4_L1() {
    return SDSMK4(8.14);
  }

  /** SDS MK4 - 6.75 : 1 */
  public static SwerveDriveConstants SDSMK4_L2() {
    return SDSMK4(6.75);
  }

  /** SDS MK4 - 6.12 : 1 */
  public static SwerveDriveConstants SDSMK4_L3() {
    return SDSMK4(6.12);
  }

  /** SDS MK4 - 5.14 : 1 */
  public static SwerveDriveConstants SDSMK4_L4() {
    return SDSMK4(5.14);
  }

  /** Swerve Drive Specialties - MK4i Module */
  public static SwerveDriveConstants SDSMK4i(double driveGearRatio) {
    double wheelDiameter = Units.inchesToMeters(4.0);
    double angleGearRatio = 150.0 / 7.0;
    /** (150 / 7) : 1 */
    boolean driveMotorInvert = false;
    boolean angleMotorInvert = true;
    boolean canCoderInvert = false;
    return new SwerveDriveConstants(
        wheelDiameter,
        angleGearRatio,
        driveGearRatio,
        driveMotorInvert,
        angleMotorInvert,
        canCoderInvert);
  }

  /** SDS MK4i - 8.14 : 1 */
  public static SwerveDriveConstants SDSMK4i_L1() {
    return SDSMK4i(8.14);
  }

  /** SDS MK4i - 6.75 : 1 */
  public static SwerveDriveConstants SDSMK4i_L2() {
    return SDSMK4i(6.75);
  }

  /** SDS MK4i - 6.12 : 1 */
  public static SwerveDriveConstants SDSMK4i_L3() {
    return SDSMK4i(6.12);
  }
}
