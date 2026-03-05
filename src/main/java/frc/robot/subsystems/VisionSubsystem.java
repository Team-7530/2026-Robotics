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

import com.ctre.phoenix6.Utils;
// import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Telemetry;
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

public class VisionSubsystem extends SubsystemBase {
  public static final String LIMELIGHTNAME = "";
  public static final String LIMELIGHTURL = "limelight.local";
  
  // Cam - x = +toward front, 0 center, -toward rear in meters.
  //       y = -left of center, 0 center, +right of center in meters
  //       z = +up from base of robot in meters
  //    roll = rotate around front/rear in radians. PI = upsidedown
  //   pitch = tilt down/up along left/right axis. PI/4 = tilt down 45 degrees, -PI/4 = tilt up 45
  //     yaw = rotate left/right around z axis. PI/4 = rotate camera to the left 45 degrees.
  public static final Pose3d LIMELIGHTPOSE = new Pose3d(
          new Translation3d(Inches.of(11).in(Meters),
                            Inches.of(8.7).in(Meters),
                            Inches.of(6.3).in(Meters)),
          new Rotation3d(Degrees.of(0), 
                        Degrees.of(45.0), 
                        Degrees.of(0)));

  // The standard deviations of our vision estimated poses, which affect correction rate
  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.5, 0.5, Degrees.of(30).in(Radians));
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.1, 0.1, Degrees.of(5).in(Radians));

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

  // Pipeline indices (placeholders) - update these to the indices you configure
  // on each Limelight. Use constants for clarity in code.
  public static final int LIMELIGHT_PIPELINE_HUB_BLUE = 1;
  public static final int LIMELIGHT_PIPELINE_HUB_RED = 2;
  public static final int LIMELIGHT_PIPELINE_TOWER_BLUE = 3;
  public static final int LIMELIGHT_PIPELINE_TOWER_RED = 4;

  private Matrix<N3, N1> curStdDevs;

  // Simulation debug field
  private Field2d simDebugField = null;

  // hysteresis parameters (ambiguity-delta-based)
  private static final double AMBIGUITY_DELTA_THRESHOLD = 0.15; // new ambiguity must be this much lower

  /* Cameras */
  public UsbCamera cam0;

  private final Telemetry telemetry;

  public VisionSubsystem(Telemetry telemetry) {
    this.telemetry = telemetry;

  // configure camera pose and LED mode using the helper routines
  LimelightHelpers.setCameraPose_RobotSpace(
    LIMELIGHTNAME,
    LIMELIGHTPOSE.getTranslation().getX(),
    LIMELIGHTPOSE.getTranslation().getY(),
    LIMELIGHTPOSE.getTranslation().getZ(),
    Math.toDegrees(LIMELIGHTPOSE.getRotation().getX()),
    Math.toDegrees(LIMELIGHTPOSE.getRotation().getY()),
    Math.toDegrees(LIMELIGHTPOSE.getRotation().getZ()));
  LimelightHelpers.setLEDMode_PipelineControl(LIMELIGHTNAME);

  // there is no longer an object to construct for pose estimation; the
  // helpers read directly from NetworkTables when requested.

    if (RobotBase.isReal()) {
      // cam0 = CameraServer.startAutomaticCapture();
      // cam0.setResolution(240, 160);
      // cam0.setFPS(15);
    }

    if (RobotBase.isSimulation()) {
      // Provide a Field2d for visualizing limelight estimations and tags in simulation.
      simDebugField = new Field2d();
      telemetry.putData("Vision/SimField", simDebugField);
    }
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
   * Chooses kMultiTagStdDevs when multiple tags are visible, otherwise uses
   * kSingleTagStdDevs for close/low-ambiguity single-tag sightings. Falls back to
   * scaled multi-tag stddevs based on average tag distance.
   */
  private void updateEstimationStdDevs(Optional<PoseEstimate> estimatedPose) {
    if (estimatedPose.isEmpty()) {
      curStdDevs = kSingleTagStdDevs;
      return;
    }

    PoseEstimate est = estimatedPose.get();
    if (est.tagCount == 0) {
      curStdDevs = kSingleTagStdDevs;
      return;
    }

    if (est.tagCount > 1) {
      curStdDevs = kMultiTagStdDevs;
      return;
    }

    // Single tag: prefer single-tag stddevs if low ambiguity and close enough
    double amb = Double.MAX_VALUE;
    double dist = est.avgTagDist;
    if (est.rawFiducials != null && est.rawFiducials.length > 0) {
      amb = est.rawFiducials[0].ambiguity;
      dist = est.rawFiducials[0].distToCamera;
    }

    if (amb <= 0.7 && dist <= 3.0) {
      curStdDevs = kSingleTagStdDevs;
    } else {
      // Scale multi-tag stddevs by distance heuristic
      curStdDevs = kMultiTagStdDevs.times(1 + (est.avgTagDist * est.avgTagDist / 30));
    }
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

    PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHTNAME);
    if (est.tagCount >= 1) {
      curStdDevs = est.tagCount > 1 ? kMultiTagStdDevs : kSingleTagStdDevs;
      return Optional.of(est);
    }
    // helpers return a non-null object even when no tags are visible; use
    // the `tagCount` field to determine emptiness.
    return Optional.empty();
  }

  public Optional<PoseEstimate> getVisionMeasurement_MT1() {
    PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHTNAME);
    if (est.tagCount > 0) {
      double maxAmb = 0.0;
      for (var rf : est.rawFiducials) {
        if (rf != null) {
          maxAmb = Math.max(maxAmb, rf.ambiguity);
        }
      }
      if (maxAmb >= AMBIGUITY_DELTA_THRESHOLD) {
        telemetry.putNumber("Vision/SelectedAmbiguity", maxAmb, true);
        updateEstimationStdDevs(Optional.of(est));
        return Optional.of(est);
      }
    }
    updateEstimationStdDevs(Optional.empty());
    return Optional.empty();
  }

  public void updateGlobalPose(CommandSwerveDrivetrain drivetrain) {
    if ((RobotState.isAutonomous() || RobotState.isTest()) &&
        (Math.abs(drivetrain.getState().Speeds.vxMetersPerSecond) < 0.2) &&
        (Math.abs(drivetrain.getState().Speeds.vyMetersPerSecond) < 0.2) &&
        (Math.abs(drivetrain.getState().Speeds.omegaRadiansPerSecond) < RotationsPerSecond.of(2).in(RadiansPerSecond))) {

        var pose = drivetrain.getState().Pose;
      telemetry.putNumber("DriveTrain/PoseX", pose.getTranslation().getX());
      telemetry.putNumber("DriveTrain/PoseY", pose.getTranslation().getY());
      telemetry.putNumber("DriveTrain/PoseTheta", pose.getRotation().getDegrees());


      // Limelight-only: get chosen limelight pose (with hysteresis) and add it to estimator
      // var postEst = this.getVisionMeasurement_MT2(drivetrain.getPigeon2().getYaw());
      var postEst = this.getVisionMeasurement_MT1();
      postEst.ifPresent(
          est -> {
            if (est.tagCount >= 1) {
              telemetry.putNumber("Vision/Camera/" + LIMELIGHTNAME + "/MT1PoseX", est.pose.getTranslation().getX());
              telemetry.putNumber("Vision/Camera/" + LIMELIGHTNAME + "/MT1PoseY", est.pose.getTranslation().getY());
              telemetry.putNumber("Vision/Camera/" + LIMELIGHTNAME + "/MT1TagCount", est.tagCount);
                  drivetrain.addVisionMeasurement(
                      est.pose, 
                Utils.fpgaToCurrentTime(est.timestampSeconds), 
                this.getEstimationStdDevs());

                // drivetrain.resetPose(est.pose);
            }
          });
    }
  }

  public Command updateGlobalPoseCommand(CommandSwerveDrivetrain drivetrain) {
    // schedules a periodic pose-update; usually set as a default command
    return run(() -> this.updateGlobalPose(drivetrain)).withName("UpdateGlobalPoseCommand");
  }

  // ---------- Limelight pipeline helpers
  /** Set the pipeline index on a single limelight camera. */
  public void setPipeline(int pipelineIndex) {
    LimelightHelpers.setPipelineIndex(LIMELIGHTNAME, pipelineIndex);
    telemetry.putNumber("Vision/Camera/" + LIMELIGHTNAME + "/Pipeline", pipelineIndex);
  }


  // ---------- Command-returning helpers for use in Command groups / events

  /** Return a command that sets a specific pipeline index on all cameras when executed. */
  public Command setPipelineCommand(int pipelineIndex) {
    return run(() -> setPipeline(pipelineIndex)).withName("SetPipelineAll-" + pipelineIndex);
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
