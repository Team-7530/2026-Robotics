package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
// AprilTag classes live in the top-level WPILib package, not math
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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

    /** X/Y offset from a tag location to the physical centre of the hub. */
      //------------------------------------------------------------------
      // AprilTag-based field layouts
      //----------------------------------------------------------------------
      // FRC provides two JSON layouts for the 2026 field (``Rebuilt'' and
      // ``Rebuilt_Andymark'') because the physical coordinates differ slightly
      // depending on the vendor.  Pick the appropriate layout for your event by
      // setting ACTIVE_LAYOUT below.  You can also override this at runtime by
      // replacing the `apriltagLayout` reference if you load a different file.
      public enum FieldLayout {
        REBUILT,
        REBUILT_ANDYMARK
      }

      // change this constant to switch between the two supplied layouts
      public static final FieldLayout ACTIVE_LAYOUT = FieldLayout.REBUILT;

      // The common WPILib helper that holds poses for every tag on the field
      private static final AprilTagFieldLayout apriltagLayout;

      static {
        switch (ACTIVE_LAYOUT) {
          case REBUILT:
            apriltagLayout = 
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
            break;
          case REBUILT_ANDYMARK:
            apriltagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
            break;
          default:
            throw new IllegalStateException("Unknown layout " + ACTIVE_LAYOUT);
        }
      }

      // IDs of the two hub tags; update these values to match the game document.
      public static final int RED_HUB_TAG_ID = 10;
      public static final int BLUE_HUB_TAG_ID = 20;

      /**
       * Offset from a tag coordinate to the physical centre of the hub fixture.
       * The WPILib field layouts give you the tag locations; the mechanical
       * centre is a fixed translation from that point.
       * (Multi-AprilTags are 0.1778 apart, and 0.425958 from the field interior).  
       * Teams can adjust this value based on their own measurements.
       */
      public static final Translation2d HUB_CENTRE_OFFSET =
          new Translation2d(0.603758, 0); 

      /**
       * Return the pose of the hub centre for the given alliance.  The result is
       * computed from the appropriate april-tag pose found in the currently
       * selected `apriltagLayout` plus the constant offset above.
       */
      public static Pose2d getHubPose(boolean isBlue) {
        int tagId = isBlue ? BLUE_HUB_TAG_ID : RED_HUB_TAG_ID;
        var maybePose = apriltagLayout.getTagPose(tagId);
        if (maybePose.isEmpty()) {
          // fallback to origin so code still compiles/run; real layout should
          // always contain the requested tag.
          return new Pose2d();
        }
        Pose2d tagPose = maybePose.get().toPose2d();
        return new Pose2d(
            tagPose.getTranslation().plus(HUB_CENTRE_OFFSET), tagPose.getRotation());
    }
  }
}
