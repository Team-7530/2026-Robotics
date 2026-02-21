/* VisionSubsystem
 *
 * This file contains the limelight-based vision subsystem and selection/hysteresis logic
 * for choosing the best limelight pose estimate. PhotonVision artifacts were previously
 * used and have been removed — simulation and pose estimation now use limelight helpers
 * and a Field2d debug field.
 */

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Utils;
// import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Telemetry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {
  public static final String LIMELIGHTNAME = "limelight";
  public static final String LIMELIGHTURL = "limelight.local";
  
  // Optional second limelight (e.g. limelight2). Update name/URL if you have a second camera.
  public static final String LIMELIGHTNAME_2 = "limelight-two";
  public static final String LIMELIGHTURL_2 = "limelight-two.local";

  // Cam - x = +toward front, 0 center, -toward rear in meters.
  //       y = +left of center, 0 center, -right of center in meters
  //       z = +up from base of robot in meters
  //    roll = rotate around front/rear in radians. PI = upsidedown
  //   pitch = tilt down/up along left/right axis. PI/4 = tilt down 45 degrees, -PI/4 = tilt up 45
  //     yaw = rotate left/right around z axis. PI/4 = rotate camera to the left 45 degrees.
  public static final List<Pair<String, Transform3d>> kCamerasList =
      List.of(
          // Pair.of("OV9281", new Transform3d(new Translation3d(0.28, 0, 0.15), new Rotation3d(0, 0, 0))),
        Pair.of(
          LIMELIGHTNAME,
          new Transform3d(new Translation3d(0.28, 0, 0.16), new Rotation3d(0, 0, 0)))
        // Pair.of(
        //   LIMELIGHTNAME_2,
        //   new Transform3d(new Translation3d(0.28, 0.10, 0.16), new Rotation3d(0, 0, 0)))
      );

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(2, 2, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 2);

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

  private final List<Limelight> limelightCameras = new ArrayList<>();
  private final List<LimelightPoseEstimator> poseEstimators = new ArrayList<>();
  private Matrix<N3, N1> curStdDevs;

  // Simulation debug field
  private Field2d simDebugField = null;

  // Hysteresis / selection state for choosing which limelight camera's estimate to use
  private String lastSelectedCameraName = "";
  // hysteresis parameters (ambiguity-delta-based)
  private double lastSelectedAmbiguity = Double.MAX_VALUE;
  private PoseEstimate lastChosenPoseEstimate = null;
  private double switchConfidence = 0.0; // grows when candidate is meaningfully better
  private static final double AMBIGUITY_DELTA_THRESHOLD = 0.15; // new ambiguity must be this much lower
  private static final double CONFIRM_THRESHOLD = 3.0; // confidence needed to commit a switch
  private static final double CONFIDENCE_INCREMENT = 1.0; // per-frame increment when condition met
  private static final double CONFIDENCE_DECREMENT = 0.5; // per-frame decrement when condition not met

  /* Cameras */
  public UsbCamera cam0;

  private final Telemetry telemetry;

  public VisionSubsystem(Telemetry telemetry) {
    this.telemetry = telemetry;

    Limelight limelight1 = new Limelight(LIMELIGHTNAME);

    limelight1.getSettings()
      .withLimelightLEDMode(LEDMode.PipelineControl)
      .withCameraOffset(new Pose3d(Inches.of(11).in(Meters),
                                    Inches.of(0).in(Meters),
                                    Inches.of(6.3).in(Meters),
                                    Rotation3d.kZero))
      .save();
    limelightCameras.add(limelight1);
    poseEstimators.add(limelight1.createPoseEstimator(EstimationMode.MEGATAG2));

    // Limelight limelight2 = new Limelight(LIMELIGHTNAME_2);
    // limelight2.getSettings()
    //   .withLimelightLEDMode(LEDMode.PipelineControl)
    //   .withCameraOffset(new Pose3d(Inches.of(11).in(Meters),
    //                                 Inches.of(0).in(Meters),
    //                                 Inches.of(6.3).in(Meters),
    //                                 new Rotation3d(0, 0, Degrees.of(180.0).in(Radian))))
    //   .save();
    // limelightCameras.add(limelight2);
    // poseEstimators.add(limelight2.createPoseEstimator(EstimationMode.MEGATAG2));

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

  public Optional<PoseEstimate> getVisionMeasurement_MT2(double yawdegrees) {
    Orientation3d robotOrientation = new Orientation3d(new Rotation3d(0, 0, Degrees.of(yawdegrees).in(Radians)),
      new AngularVelocity3d(DegreesPerSecond.of(0),
                            DegreesPerSecond.of(0),
                            DegreesPerSecond.of(0)));

    for (Limelight limelight : limelightCameras) {
      limelight.getSettings()
        .withRobotOrientation(robotOrientation)
        .save();
    }

    for (LimelightPoseEstimator poseEst : poseEstimators) {
      Optional<PoseEstimate> pose = poseEst.getPoseEstimate();
      
      if (pose.isPresent() && pose.get().tagCount >= 1) {
        curStdDevs = pose.get().tagCount > 1 ? kMultiTagStdDevs : kSingleTagStdDevs;
        return Optional.of(pose.get());
      }
    }
    return Optional.empty();
  }

  public Optional<PoseEstimate> getVisionMeasurement_MT1() {
    // Collect pose estimates and per-camera ambiguity
    List<PoseEstimate> candidates = new ArrayList<>();
    List<String> names = new ArrayList<>();

    for (int i = 0; i < poseEstimators.size(); ++i) {
      LimelightPoseEstimator poseEst = poseEstimators.get(i);
      Optional<PoseEstimate> pose = poseEst.getPoseEstimate();

      if (!pose.isPresent()) continue;

      double minAmb = Double.MAX_VALUE;
      if (pose.get().rawFiducials != null && pose.get().rawFiducials.length > 0) {
        for (var rf : pose.get().rawFiducials) {
          if (rf == null) continue;
          minAmb = Math.min(minAmb, rf.ambiguity);
        }
      }

      // record candidate and the camera name that produced it
      candidates.add(pose.get());
      names.add(limelightCameras.get(i).limelightName);
      // SmartDashboard.putNumber("Vision/Camera/" + limelightCameras.get(i).limelightName + "/Ambiguity", minAmb == Double.MAX_VALUE ? -1 : minAmb);
    }

  if (candidates.isEmpty()) {
    updateEstimationStdDevs(Optional.empty());
    return Optional.empty();
  }

    // Choose best candidate by lowest ambiguity, tie-break by tagCount then avgTagDist
    int bestIndex = 0;
    double bestAmb = Double.MAX_VALUE;
    for (int i = 0; i < candidates.size(); ++i) {
      var p = candidates.get(i);
      double minAmb = Double.MAX_VALUE;
      if (p.rawFiducials != null && p.rawFiducials.length > 0) {
        for (var rf : p.rawFiducials) {
          if (rf == null) continue;
          minAmb = Math.min(minAmb, rf.ambiguity);
        }
      }

      if (minAmb < bestAmb) {
        bestAmb = minAmb;
        bestIndex = i;
      } else if (minAmb == bestAmb) {
        if (p.tagCount > candidates.get(bestIndex).tagCount) bestIndex = i;
        else if (p.tagCount == candidates.get(bestIndex).tagCount
            && p.avgTagDist < candidates.get(bestIndex).avgTagDist) bestIndex = i;
      }
    }

    var bestPose = candidates.get(bestIndex);
    String candidateName = names.get(bestIndex);

    // Ambiguity-delta-based hysteresis
    if (lastChosenPoseEstimate == null || lastSelectedCameraName.isEmpty()) {
      lastChosenPoseEstimate = bestPose;
      lastSelectedCameraName = candidateName;
      lastSelectedAmbiguity = bestAmb;
      switchConfidence = 0.0;
    } else {
      double delta = lastSelectedAmbiguity - bestAmb;
      boolean candidateBetter = delta >= AMBIGUITY_DELTA_THRESHOLD;
      if (candidateBetter) {
        switchConfidence = Math.min(CONFIRM_THRESHOLD, switchConfidence + CONFIDENCE_INCREMENT);
      } else {
        switchConfidence = Math.max(0.0, switchConfidence - CONFIDENCE_DECREMENT);
      }

      double progress = switchConfidence / CONFIRM_THRESHOLD;
  telemetry.putString("Vision/PendingSwitchTo", candidateName);
  telemetry.putNumber("Vision/SwitchConfidence", progress);

      if (switchConfidence >= CONFIRM_THRESHOLD) {
        // commit
        lastChosenPoseEstimate = bestPose;
        lastSelectedCameraName = candidateName;
        lastSelectedAmbiguity = bestAmb;
        switchConfidence = 0.0;
        telemetry.putString("Vision/PendingSwitchTo", "");
        telemetry.putNumber("Vision/SwitchConfidence", 1.0);
      }
    }

    // Update stddevs and return the currently chosen pose estimate
    updateEstimationStdDevs(Optional.of(lastChosenPoseEstimate));
  telemetry.putString("Vision/SelectedCamera", lastSelectedCameraName == null ? "" : lastSelectedCameraName);
  telemetry.putNumber("Vision/SelectedAmbiguity", lastSelectedAmbiguity == Double.MAX_VALUE ? -1 : lastSelectedAmbiguity);

    return Optional.of(lastChosenPoseEstimate);
  }

  public void updateGlobalPose(CommandSwerveDrivetrain drivetrain) {
    if ((RobotState.isAutonomous() || RobotState.isTest()) &&
        (Math.abs(drivetrain.getState().Speeds.vxMetersPerSecond) < 0.2) &&
        (Math.abs(drivetrain.getState().Speeds.vyMetersPerSecond) < 0.2) &&
        (Math.abs(drivetrain.getState().Speeds.omegaRadiansPerSecond) < RotationsPerSecond.of(2).in(RadiansPerSecond))) {

      // Limelight-only: get chosen limelight pose (with hysteresis) and add it to estimator
      var limelightEst = this.getVisionMeasurement_MT1();
      limelightEst.ifPresent(
          est -> {
            if (est.tagCount >= 1) {
        drivetrain.addVisionMeasurement(
          est.pose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds), this.getEstimationStdDevs());
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
  public void setPipelineForFrontCamera(int pipelineIndex) {
    limelightCameras.get(0).getSettings().withPipelineIndex(pipelineIndex).save();
    telemetry.putNumber("Vision/Camera/" + LIMELIGHTNAME + "/Pipeline", pipelineIndex);
  }

  public void setPipelineForBackCamera(int pipelineIndex) {
    limelightCameras.get(1).getSettings().withPipelineIndex(pipelineIndex).save();
    telemetry.putNumber("Vision/Camera/" + LIMELIGHTNAME_2 + "/Pipeline", pipelineIndex);
  }

  /** Set the pipeline index for every configured limelight camera. */
  public void setPipelineForAllCameras(int pipelineIndex) {
    for (Limelight camera : limelightCameras) {
      camera.getSettings().withPipelineIndex(pipelineIndex).save();
      telemetry.putNumber("Vision/Camera/" + camera.limelightName + "/Pipeline", pipelineIndex);
    }
  }

  /**
   * Convenience helper: set the pipeline that filters HUB tags for the current alliance.
   * Pass true for blue alliance, false for red. Indices are defined in
   * {@link frc.robot.Constants.Vision}.
   */
  public void setHubPipelineForAlliance(boolean isBlue) {
    int idx = isBlue ? LIMELIGHT_PIPELINE_HUB_BLUE : LIMELIGHT_PIPELINE_HUB_RED;
    setPipelineForAllCameras(idx);
    telemetry.putString("Vision/ActiveMode", "HubPipeline");
  }

  /**
   * Convenience helper: set the pipeline that filters Tower tags for the current alliance.
   * Pass true for blue alliance, false for red.
   */
  public void setTowerPipelineForAlliance(boolean isBlue) {
    int idx = isBlue ? LIMELIGHT_PIPELINE_TOWER_BLUE : LIMELIGHT_PIPELINE_TOWER_RED;
    setPipelineForAllCameras(idx);
    telemetry.putString("Vision/ActiveMode", "TowerPipeline");
  }

  // ---------- Command-returning helpers for use in Command groups / events

  /** Return a command that sets a specific pipeline index on all cameras when executed. */
  public Command setPipelineForAllCamerasCommand(int pipelineIndex) {
    return run(() -> setPipelineForAllCameras(pipelineIndex)).withName("SetPipelineAll-" + pipelineIndex);
  }

  /** Return a command that selects Hub pipeline based on the DriverStation alliance at runtime. */
  public Command setHubPipelineCommand() {
    return run(
            () ->
        setHubPipelineForAlliance(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue))
        .withName("SetHubPipeline");
  }

  /** Return a command that selects Tower pipeline based on the DriverStation alliance at runtime. */
  public Command setTowerPipelineCommand() {
    return run(
            () ->
        setTowerPipelineForAlliance(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue))
        .withName("SetTowerPipeline");
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
