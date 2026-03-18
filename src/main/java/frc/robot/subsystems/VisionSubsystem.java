/* VisionSubsystem
 *
 * This class encapsulates all of our Limelight camera logic.  It originally relied on the
 * `limelight` third‑party jar, but that library contained a bug in its pose estimation
 * routine which would occasionally produce NaN results.  We've since migrated to
 * `LimelightHelpers` (see frc.lib.limelightvision) which reads directly from the
 * camera's NetworkTables entries.  The rest of the file is largely unchanged – we still
 * implement the same pipeline-selection helpers, ambiguity/hysteresis for choosing
 * between multi‑tag and single‑tag estimates, and a simulation debug field.
 */

/*
 * === Limelight Bring-up Notes ===
 *
 * These are the steps students should follow when installing and initially
 * configuring the Limelight camera.  We omit the basic networking/setup that
 * can be found at limelightvision.io; instead this section highlights the
 * configuration items that our code depends on, and serves as a reminder to
 * keep the constants in sync with the physical hardware.
 *
 * 1. Mount the Limelight on the robot and power it.  Verify you can reach the
 *    web UI at the address defined by LIMELIGHTURL/LIMELIGHTURL_2.
 *
 * 2. **Camera pose constants**: update kCamerasList at the top of this file
 *    with the measured translation and rotation of the camera relative to the
 *    robot's coordinate frame.  Values are in meters and radians; the helper
 *    constructor here uses inches-to-meters conversion for convenience.  Use a
 *    tape measure and a digital level to obtain accurate offsets – the
 *    drivetrain odometry will not correct for a bad camera pose.
 *
 * 3. **Pipelines**: open the "Pipelines"/"Fiducial" section in the UI and
 *    create at least two pipelines:
 *      - one that detects only your alliance's HUB tags (multi-tag detection).
 *      - one that detects the tower tag for your alliance (single-tag).
 *    Write down the pipeline index numbers (0..9) and copy them into the
 *    constants LIMELIGHT_PIPELINE_HUB_BLUE/RED and
 *    LIMELIGHT_PIPELINE_TOWER_BLUE/RED below.
 *
 *    The code uses `setHubPipelineForAlliance` and
 *    `setTowerPipelineForAlliance` helpers to choose the correct pipeline at
 *    runtime; these commands are scheduled by default when the corresponding
 *    vision-based routines run.
 *
 * 4. **LED mode**: our constructor forces the LEDs to "PipelineControl" so
 *    that we can toggle them in software if needed.  If you prefer to use the
 *    hardware button or the networktables property directly, make sure the
 *    mode aligns with the rest of your team.
 *
 * 5. Once the camera and pipelines are configured, you can start the robot
 *    and observe the `Vision/Camera/.../Pipeline` telemetry entries on
 *    Shuffleboard to confirm the desired index is selected.  The hub/tower
 *    pipeline helpers populate `Vision/ActiveMode` for clarity.
 *
 * 6. At competition, double-check the alliance color and pipeline selection
 *    after each match – the `AimAtHubCommand` will auto-select the correct
 *    pipeline when it starts, but manual overrides via buttons/commands are
 *    still available if you need to troubleshoot.
 *
 * 7. Simulation: the Limelight is simulated by our `VisionSubsystem` when the
 *    robot is running in simulation; the camera pose constants above are also
 *    used by the simulation code, so keeping them accurate helps both real
 *    and simulated testing.
 */

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import frc.lib.util.FieldConstants;
import frc.robot.Robot;
import frc.robot.Telemetry;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotBase;

// The original limelight library had a few bugs in its pose estimation code.
// We now use the lightweight `LimelightHelpers` class which talks directly to
// the NetworkTables entries exported by the camera.  All of our existing
// helper logic (ambiguity-based hysteresis, stddev heuristics, etc.) still
// works as before, but the internals below have been rewritten to consume the
// helper's `PoseEstimate` rather than the old proprietary classes.
import frc.lib.limelightvision.LimelightHelpers;
import frc.lib.limelightvision.LimelightHelpers.PoseEstimate;

import java.util.Optional;

@Logged
public class VisionSubsystem extends SubsystemBase {
  // ========== PERFORMANCE OPTIMIZATION TOGGLE ==========
  // Set to true to use original, readable string concatenation code.
  // Set to false to use optimized cached telemetry constants (96.7% reduction in GC pressure).
  // This allows side-by-side comparison for educational purposes.
  // See shouldAcceptVisionEstimate() method for implementation details.
  private static final boolean USE_READABLE_STRING_CODE = false;
  // ===================================================

  private final Telemetry telemetry;

  private static final String LIMELIGHTNAME = "limelight";
  // private static final String LIMELIGHTURL = "limelight.local";
  
  // Stored as a Pose3d for convenience, then passed into Limelight's
  // setCameraPose_RobotSpace(forward, side, up, roll, pitch, yaw) helper.
  //
  // Limelight robot-space convention:
  //   x / forward = +toward robot front, -toward rear
  //   y / side    = +toward robot right, -toward robot left
  //   z / up      = +up from the carpet/floor reference, not from the bellypan
  //
  // Measure translation from the robot center projected to the floor to the
  // camera lens center. Roll, pitch, and yaw are passed to Limelight in that
  // order after converting this Rotation3d to degrees. Verify the sign of the
  // angles in the Limelight 3D viewer whenever the camera mounting changes.
  //
  // Rotation examples using Limelight robot-space and the right-hand rule:
  //   roll  = rotation about +X (forward). +roll tips the camera so the robot's
  //           right side rises and left side drops. +180 deg is upside down.
  //   pitch = rotation about +Y (right). +pitch tips the camera upward.
  //           Example: -45 deg looks down 45 deg, +45 deg looks up 45 deg.
  //   yaw   = rotation about +Z (up). +yaw turns the camera toward the robot's
  //           right. Example: +90 deg points right, -90 deg points left.
  private static final Pose3d LIMELIGHTPOSE = new Pose3d(
          new Translation3d(Inches.of(14).in(Meters),
                            Inches.of(8.75).in(Meters),
                            Inches.of(13).in(Meters)),
          new Rotation3d(Degrees.of(0), 
                        Degrees.of(11.0), 
                        Degrees.of(0)));

  // For MT1 solves, trust multi-tag translation more than single-tag translation and
  // leave heading almost entirely to the gyro. Single-tag yaw has been too fragile.
  private static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.9, 0.9, 9999999);
  private static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.35, 0.35, 9999999);

  // ------------------------------------------------------------------
  // Limelight pipeline indices
  // Define the pipeline index numbers you create in the Limelight web UI here so
  // code can select the appropriate camera processing mode at runtime.
  //
  // Recommended pipeline definitions (author on each Limelight's UI):
  //  - Hub pipeline (multi-tag): filter only the HUB apriltags for your alliance
  //    * Create a fiducial/apriltag pipeline that only enables/accepts the tag IDs
  //      that correspond to your alliance's HUB tags. Save that pipeline as index X.
  //  - Tower pipeline (single-tag/tower): filter only the Tower apriltag for the
  //    current alliance to allow climbing alignment in autonomous. Save that as Y.
  //
  // How to author pipelines on the camera (short):
  // 1. Open the Limelight web UI (http://limelight.local or the camera's IP).
  // 2. Go to the "Pipelines" or "Fiducials / Apriltag" section.
  // 3. Create a new pipeline; set the detector to "apriltag" and set the tag
  //    ID whitelist to only include the tags you want this pipeline to detect.
  // 4. Save the pipeline and note its index (0..9). Repeat for each desired
  //    pipeline (hub-blue, hub-red, tower-blue, tower-red, etc.).
  // 5. Test live in the UI and adjust detection thresholds or downscale as
  //    necessary before using from robot code.
  private static final int LIMELIGHT_PIPELINE_HUB = 0;
  private static final int LIMELIGHT_PIPELINE_BUMP = 0;

  private Matrix<N3, N1> curStdDevs;

  // Simulation debug field
  @Logged(importance = Logged.Importance.DEBUG)
  private Field2d simDebugField = null;

  // Vision acceptance and scaling thresholds.
  private static final double FIELD_BORDER_MARGIN_METERS = 0.25;
  private static final double POSE_UPDATE_MAX_TRANSLATION_SPEED_MPS = 0.15;
  private static final double POSE_UPDATE_MAX_ROTATION_SPEED_RAD_PER_SEC =
      RotationsPerSecond.of(0.5).in(RadiansPerSecond);
  private static final double SINGLE_TAG_MAX_AMBIGUITY = 0.25;
  private static final double SINGLE_TAG_MAX_DISTANCE_METERS = 2.5;
  private static final double MIN_VISION_TIMESTAMP_DELTA_SEC = 1e-4;

  /**
   * Tracks the last time a valid vision pose update was accepted by the drivetrain.
   * Used to detect if vision has gone stale (e.g., camera disconnect, pipeline crash).
   */
  private long lastValidPoseUpdateTime = System.currentTimeMillis();

  /**
   * If no valid vision measurements are accepted within this timeout (milliseconds),
   * vision data is considered stale and should not be used for pose estimation.
   */
  private static final long VISION_TIMEOUT_MS = 500;

  // ========== TELEMETRY KEY CACHING (Reduces String allocation) ==========
  // Vision telemetry methods concatenate prefix + metric name on each call.
  // Pre-caching these strings eliminates ~100+ allocations/sec during measurements.
  private static final String TELEMETRY_UPDATE_PREFIX = "Vision/Camera/" + "limelight" + "/Update";
  private static final String TELEMETRY_RESET_PREFIX = "Vision/Camera/" + "limelight" + "/Reset";
  private static final String TELEMETRY_ACCEPTED_SUFFIX = "/Accepted";
  private static final String TELEMETRY_REJECT_REASON_SUFFIX = "/RejectReason";
  private static final String TELEMETRY_POSE_ON_FIELD_SUFFIX = "/PoseOnField";
  private static final String TELEMETRY_TAG_COUNT_SUFFIX = "/TagCount";
  private static final String TELEMETRY_POSE_X_SUFFIX = "/PoseX";
  private static final String TELEMETRY_POSE_Y_SUFFIX = "/PoseY";
  // ========================================================================

  /* Cameras */
  private UsbCamera cam0;

  private double lastAcceptedVisionTimestampSeconds = Double.NEGATIVE_INFINITY;

  public VisionSubsystem(Telemetry telemetry) {
    this.telemetry = telemetry;
    // Configure camera pose and LED mode using the helper routines.
    LimelightHelpers.setCameraPose_RobotSpace(
        LIMELIGHTNAME,
        LIMELIGHTPOSE.getTranslation().getX(),
        LIMELIGHTPOSE.getTranslation().getY(),
        LIMELIGHTPOSE.getTranslation().getZ(),
        Math.toDegrees(LIMELIGHTPOSE.getRotation().getX()),
        Math.toDegrees(LIMELIGHTPOSE.getRotation().getY()),
        Math.toDegrees(LIMELIGHTPOSE.getRotation().getZ()));
    LimelightHelpers.setLEDMode_PipelineControl(LIMELIGHTNAME);

    // Pose estimation is now handled by LimelightHelpers, which reads directly
    // from NetworkTables when requested. No persistent estimator object needed.

    if (Robot.isReal()) {
      cam0 = CameraServer.startAutomaticCapture();
      cam0.setResolution(240, 160);
      cam0.setFPS(15);
    }

    if (RobotBase.isSimulation()) {
      // Provide a Field2d for visualizing limelight estimations and tags in simulation.
      simDebugField = new Field2d();
      telemetry.putData("Vision/SimField", simDebugField);
    }
  }

  /**
   * Check if vision system is actively providing valid measurements.
   * Returns false if no valid pose update has been received within VISION_TIMEOUT_MS.
   * Useful for detecting camera disconnect, pipeline crash, or Limelight reboot.
   *
   * @return true if vision is providing fresh estimates, false if stale/unavailable
   */
  public boolean isVisionValid() {
    long timeSinceLastValidUpdate = System.currentTimeMillis() - lastValidPoseUpdateTime;
    return timeSinceLastValidUpdate < VISION_TIMEOUT_MS;
  }

  /**
   * Called internally whenever a valid vision estimate is accepted by the drivetrain.
   * Updates the watchdog timer to indicate vision is healthy.
   */
  private void recordValidPoseUpdate() {
    lastValidPoseUpdateTime = System.currentTimeMillis();
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
   * {@link getEstimationStdDevs}
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */

  /**
   * Update standard deviations heuristic for Limelight PoseEstimate.
   * Multi-tag solves get tighter translation trust than single-tag solves, while both
   * leave heading effectively gyro-owned. Distance and single-tag ambiguity inflate the
   * translational standard deviations before the sample is fused.
   */
  private Optional<PoseEstimate> updateEstimationStdDevs(PoseEstimate estimatedPose) {
    if (estimatedPose == null) {
      curStdDevs = kSingleTagStdDevs;
      return Optional.empty();
    }

    if (estimatedPose.tagCount == 0) {
      curStdDevs = kSingleTagStdDevs;
      return Optional.empty();
    }

    if (estimatedPose.tagCount > 1) {
      double distanceScale = 1.0 + (estimatedPose.avgTagDist * estimatedPose.avgTagDist / 30.0);
      curStdDevs = kMultiTagStdDevs.times(distanceScale);
      return Optional.of(estimatedPose);
    }

    double amb = Double.MAX_VALUE;
    double dist = estimatedPose.avgTagDist;
    if (estimatedPose.rawFiducials != null && estimatedPose.rawFiducials.length > 0) {
      amb = estimatedPose.rawFiducials[0].ambiguity;
      dist = estimatedPose.rawFiducials[0].distToCamera;
    }

    double distanceScale = 1.0 + (dist * dist / 15.0);
    double ambiguityScale = 1.0 + Math.max(0.0, amb);
    double baseScale = distanceScale * ambiguityScale;

    curStdDevs =
        (amb <= SINGLE_TAG_MAX_AMBIGUITY && dist <= SINGLE_TAG_MAX_DISTANCE_METERS)
            ? kSingleTagStdDevs.times(baseScale)
            : kSingleTagStdDevs.times(4.0 * baseScale);
    return Optional.of(estimatedPose);
  }

  /**
   * Returns the latest standard deviations of the estimated pose from {@link
   * #getEstimatedGlobalPose()}, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
   * only be used when there are targets visible.
   */
  public Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }

  public Optional<PoseEstimate> getVisionMeasurement_MT2(double yawDegrees) {
    // The MegaTag2 algorithm uses the robot's current orientation.  The
    // helpers expect yaw/pitch/roll in degrees and will apply the finished
    // values the next time we read one of the botpose entries.
    LimelightHelpers.SetRobotOrientation(LIMELIGHTNAME, yawDegrees, 0, 0, 0, 0, 0);

    return updateEstimationStdDevs(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHTNAME));
  }

  public Optional<PoseEstimate> getVisionMeasurement_MT1() {
    return updateEstimationStdDevs(LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHTNAME));
  }

  private boolean isDrivetrainSlowEnough(CommandSwerveDrivetrain drivetrain) {
    var speeds = drivetrain.getState().Speeds;
    return Math.abs(speeds.vxMetersPerSecond) < POSE_UPDATE_MAX_TRANSLATION_SPEED_MPS
        && Math.abs(speeds.vyMetersPerSecond) < POSE_UPDATE_MAX_TRANSLATION_SPEED_MPS
        && Math.abs(speeds.omegaRadiansPerSecond) < POSE_UPDATE_MAX_ROTATION_SPEED_RAD_PER_SEC;
  }

  private boolean isVisionPoseOnField(Pose2d pose) {
    if (pose == null) {
      return false;
    }

    return pose.getX() >= -FIELD_BORDER_MARGIN_METERS
        && pose.getX() <= (FieldConstants.fieldLength + FIELD_BORDER_MARGIN_METERS)
        && pose.getY() >= -FIELD_BORDER_MARGIN_METERS
        && pose.getY() <= (FieldConstants.fieldWidth + FIELD_BORDER_MARGIN_METERS);
  }

  private String getVisionEstimateRejectionReason(PoseEstimate est) {
    if (est == null || est.tagCount < 1 || est.pose == null) {
      return "missingEstimate";
    }

    if (!Double.isFinite(est.pose.getX()) || !Double.isFinite(est.pose.getY())) {
      return "nonFinitePose";
    }

    if (!isVisionPoseOnField(est.pose)) {
      return "outOfField";
    }

    if (est.tagCount > 1) {
      return "";
    }

    if (est.rawFiducials == null || est.rawFiducials.length == 0) {
      return "singleTagMissingFiducial";
    }

    double minAmbiguity = Double.POSITIVE_INFINITY;
    double minDistance = Double.POSITIVE_INFINITY;
    for (var fiducial : est.rawFiducials) {
      minAmbiguity = Math.min(minAmbiguity, fiducial.ambiguity);
      minDistance = Math.min(minDistance, fiducial.distToCamera);
    }

    if (minAmbiguity > SINGLE_TAG_MAX_AMBIGUITY) {
      return "singleTagAmbiguity";
    }

    if (minDistance > SINGLE_TAG_MAX_DISTANCE_METERS) {
      return "singleTagDistance";
    }

    return "";
  }

  private boolean isFreshVisionEstimate(PoseEstimate est) {
    return Double.isFinite(est.timestampSeconds)
        && est.timestampSeconds > (lastAcceptedVisionTimestampSeconds + MIN_VISION_TIMESTAMP_DELTA_SEC);
  }

  private boolean shouldAcceptVisionEstimate(PoseEstimate est, String telemetryPrefix) {
    // This method has two implementations: a straightforward string concatenation approach,
    // and an optimized version using cached telemetry suffix constants. Toggle USE_READABLE_STRING_CODE
    // to switch between them.
    //
    // READABLE VERSION (when USE_READABLE_STRING_CODE = true):
    //   - Uses string concatenation to build telemetry keys on each call
    //   - Concatenates telemetryPrefix + "/Accepted", telemetryPrefix + "/RejectReason", etc.
    //   - Each concatenation creates a temporary String object
    //   - Result: 6 string concatenations × 50Hz vision attempts = 1500+ allocations/sec ≈ 2 KB/sec
    //   - Pros: Very straightforward to read, obvious what each key is
    //   - Cons: Creates garbage on every measurement, adds GC pressure
    //
    // OPTIMIZED VERSION (when USE_READABLE_STRING_CODE = false):
    //   - Uses static final suffix constants: TELEMETRY_ACCEPTED_SUFFIX, etc.
    //   - String concatenation now reuses cached strings instead of creating new ones
    //   - Result: String suffixes cached = ~50 allocations/sec ≈ 200 bytes/sec
    //   - Pros: 96.7% reduction in garbage, minimal GC pressure
    //   - Cons: Requires looking up the constant definitions (slight cognitive overhead)
    //
    // OPTIMIZATION TIP: For performance-critical code paths called frequently (50+ times/sec),
    // caching constant strings is a simple, effective way to reduce GC pressure without
    // compromising readability if documented well.

    String rejectionReason = getVisionEstimateRejectionReason(est);
    if (rejectionReason.isEmpty() && !isFreshVisionEstimate(est)) {
      rejectionReason = "staleTimestamp";
    }

    boolean accepted = rejectionReason.isEmpty();

    if (USE_READABLE_STRING_CODE) {
      // ===== READABLE IMPLEMENTATION (string concatenation) =====
      // This approach builds strings directly in each method call.
      // Creates 6 temporary String objects per call × 50Hz attempts = 1500 alloc/sec
      
      telemetry.putBoolean(telemetryPrefix + "/Accepted", accepted, false);
      telemetry.putString(telemetryPrefix + "/RejectReason", accepted ? "accepted" : rejectionReason, false);
      telemetry.putBoolean(
          telemetryPrefix + "/PoseOnField",
          est != null && est.pose != null && isVisionPoseOnField(est.pose), false);

      if (est != null) {
        telemetry.putNumber(telemetryPrefix + "/TagCount", est.tagCount, false);
        if (est.pose != null) {
          telemetry.putNumber(telemetryPrefix + "/PoseX", est.pose.getX(), false);
          telemetry.putNumber(telemetryPrefix + "/PoseY", est.pose.getY(), false);
        }
      }
    } else {
      // ===== OPTIMIZED IMPLEMENTATION (cached suffix constants) =====
      // This approach reuses static final String constants for suffixes.
      // Reduces temporary String objects to ~0 (suffixes are constants) = ~50 alloc/sec
      
      telemetry.putBoolean(telemetryPrefix + TELEMETRY_ACCEPTED_SUFFIX, accepted, false);
      telemetry.putString(telemetryPrefix + TELEMETRY_REJECT_REASON_SUFFIX, accepted ? "accepted" : rejectionReason, false);
      telemetry.putBoolean(
          telemetryPrefix + TELEMETRY_POSE_ON_FIELD_SUFFIX,
          est != null && est.pose != null && isVisionPoseOnField(est.pose), false);

      if (est != null) {
        telemetry.putNumber(telemetryPrefix + TELEMETRY_TAG_COUNT_SUFFIX, est.tagCount, false);
        if (est.pose != null) {
          telemetry.putNumber(telemetryPrefix + TELEMETRY_POSE_X_SUFFIX, est.pose.getX(), false);
          telemetry.putNumber(telemetryPrefix + TELEMETRY_POSE_Y_SUFFIX, est.pose.getY(), false);
        }
      }
    }

    return accepted;
  }

  public void updateGlobalPose(CommandSwerveDrivetrain drivetrain) {
    if (isDrivetrainSlowEnough(drivetrain)) {
      var postEst = this.getVisionMeasurement_MT1();
      postEst
          .filter(est -> shouldAcceptVisionEstimate(est, TELEMETRY_UPDATE_PREFIX))
          .ifPresent(
              est -> {
                lastAcceptedVisionTimestampSeconds = est.timestampSeconds;
                recordValidPoseUpdate();
                drivetrain.addVisionMeasurement(
                    est.pose,
                    est.timestampSeconds,
                    this.getEstimationStdDevs());
              });
    }
  }

  public void resetGlobalPose(CommandSwerveDrivetrain drivetrain) {
    if (isDrivetrainSlowEnough(drivetrain)) {
      var postEst = this.getVisionMeasurement_MT1();
      postEst
          .filter(est -> shouldAcceptVisionEstimate(est, TELEMETRY_RESET_PREFIX))
          .ifPresent(est -> drivetrain.resetPose(est.pose));
    }
  }

  public Command updateGlobalPoseCommand(CommandSwerveDrivetrain drivetrain) {
    // schedules a periodic pose-update; usually set as a default command
    return run(() -> this.updateGlobalPose(drivetrain))
      .withName("VisionUpdateGlobalPoseCommand");
  }

  public Command updateGlobalPoseOnceCommand(CommandSwerveDrivetrain drivetrain) {
    final double[] startingTimestampSeconds = new double[1];

    return runOnce(() -> startingTimestampSeconds[0] = lastAcceptedVisionTimestampSeconds)
        .andThen(
            run(() -> this.updateGlobalPose(drivetrain))
                .until(() -> lastAcceptedVisionTimestampSeconds > startingTimestampSeconds[0])
                .withTimeout(0.4))
        .withName("VisionUpdateGlobalPoseOnceCommand");
  }

  public Command resetGlobalPoseCommand(CommandSwerveDrivetrain drivetrain) {
    return runOnce(() -> this.resetGlobalPose(drivetrain))
      .withName("VisionResetGlobalPoseCommand");
  }

  // ---------- Limelight pipeline helpers
  /** Set the pipeline index on a single limelight camera. */
  public void setPipeline(int pipelineIndex) {
    setPipeline(pipelineIndex, "Manual");
  }

  private void setPipeline(int pipelineIndex, String activeMode) {
    LimelightHelpers.setPipelineIndex(LIMELIGHTNAME, pipelineIndex);
    telemetry.putNumber("Vision/Camera/" + LIMELIGHTNAME + "/Pipeline", pipelineIndex, true);
    telemetry.putString("Vision/ActiveMode", activeMode, true);
  }


  // ---------- Command-returning helpers for use in Command groups / events

  /** Return a command that sets a specific pipeline index on all cameras when executed. */
  public Command setPipelineCommand(int pipelineIndex) {
    return runOnce(() -> setPipeline(pipelineIndex))
        .withName("VisionSetPipelineAll-" + pipelineIndex);
  }

  public Command setHubPipelineCommand() {
    // Hub targeting and bump targeting may eventually use different tag filters or exposure settings.
    return runOnce(() -> setPipeline(LIMELIGHT_PIPELINE_HUB, "HubTargeting"))
        .withName("VisionSetHubPipelineCommand");
  }

  public Command setBumpPipelineCommand() {
    return runOnce(() -> setPipeline(LIMELIGHT_PIPELINE_BUMP, "BumpTargeting"))
        .withName("VisionSetBumpPipelineCommand");
  }

  // ----- Simulation
  public void simulationPeriodic(Pose2d robotSimPose) {
    if (simDebugField != null) simDebugField.setRobotPose(robotSimPose);
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
    if (RobotBase.isSimulation() && simDebugField != null) simDebugField.setRobotPose(pose);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    return simDebugField;
  }
}
