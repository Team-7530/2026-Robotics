package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.generated.TunerConstants;

public final class Constants {

  public static final double STICK_DEADBAND = 0.02;
  public static final double TRIGGER_SPEEDFACTOR = 0.5;
  public static final double POSITION_TOLERANCE = 0.05;
  public static final CANBus CANBUS_FD = new CANBus("CANFD");
  public static final CANBus CANBUS_RIO = new CANBus("");

  public static final class DriveTrainConstants {
    // Maximum Speed - Meters per Second
    // max angular velocity - Rotations per Second
    // 3/4 of a rotation per second
    public static final ChassisSpeeds maxSpeed =
        new ChassisSpeeds(
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
            RotationsPerSecond.of(0.75).in(RadiansPerSecond));
    public static final ChassisSpeeds cruiseSpeed = maxSpeed.times(0.6);
    public static final ChassisSpeeds slowSpeed =
        new ChassisSpeeds(
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.15,
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.15,
            RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 0.2);
  }

  /** Field geometry constants. */
  public static final class Field {
    // approximate location of the center of the scoring hub relative to the field origin
    // (update these values to match your game-specific coordinates).
    // Note: Historically HUB_POSE was used as the absolute centre of the hub.
    // In our 2026 game the hub is represented by two april tags (one for each
    // alliance) and the actual centre is a small offset from the tag coordinate.
    // We therefore expose the tag coordinates for each alliance, and keep
    // HUB_OFFSET as the vector from the tag to the mechanical centre of the
    // fixture.  "getHubPose" helper below combines them.

      // IDs of the two hub tags; update these values to match the game document.
      public static final Distance FIELD_LENGTH = Meters.of(16.540988);
      public static final Distance FIELD_WIDTH = Meters.of(8.069326);
      public static final Distance FIELD_HALF_WIDTH = Meters.of(4.034663);
    
      public static final Distance HUB_X = Meters.of(4.625518);
      public static final Distance HUB_Y = Meters.of(4.034663);
      public static final Distance HUB_HEIGHT = Meters.of(1.437087);
      
      public static final Distance BUMP_X_OFFSET = Meters.of(0.0);
      public static final Distance BUMP_Y_OFFSET = Meters.of(1.52);
        
      public static final Translation2d BLUE_HUB_CENTER = new Translation2d(HUB_X, HUB_Y);
      public static final Translation2d RED_HUB_CENTER = new Translation2d(FIELD_LENGTH.minus(HUB_X), FIELD_WIDTH.minus(HUB_Y));  
      public static final Translation2d BUMP_CENTER_OFFSET = new Translation2d(BUMP_X_OFFSET, BUMP_Y_OFFSET);

      /**
       * Return the pose of the hub centre for the given alliance. 
       */
      public static Pose2d getHubPose(boolean isBlue) {
        return isBlue ? new Pose2d(BLUE_HUB_CENTER, Rotation2d.kZero) 
                      : new Pose2d(RED_HUB_CENTER, Rotation2d.kZero);
    }
  }
}
