/* VisionSubsystem
 *
 * This file contains the limelight-based vision subsystem and selection/hysteresis logic
 * for choosing the best limelight pose estimate. PhotonVision artifacts were previously
 * used and have been removed — simulation and pose estimation now use limelight helpers
 * and a Field2d debug field.
 */

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.Vision.*;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.limelightvision.LimelightHelpers;
import frc.lib.limelightvision.LimelightHelpers.PoseEstimate;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class VisionSubsystem implements Subsystem {
  private final List<String> limelightCameras = new ArrayList<>();
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

  public VisionSubsystem() {

    for (var cam : kCamerasList) {
      String name = cam.getFirst();
      Transform3d pose = cam.getSecond();

      if (name.toLowerCase().startsWith("limelight")) {
        limelightCameras.add(name);

        Rotation3d rot = pose.getRotation();
        LimelightHelpers.setCameraPose_RobotSpace(
            name, pose.getX(), pose.getY(), pose.getZ(), rot.getX(), rot.getY(), rot.getZ());
      } else {
        // Non-limelight camera entries are ignored in limelight-only mode.
      }
    }

    if (Robot.isReal()) {
      cam0 = CameraServer.startAutomaticCapture();
      cam0.setResolution(240, 160);
      cam0.setFPS(15);
    }

    // ----- Simulation
    if (Robot.isSimulation()) {
      // Provide a Field2d for visualizing limelight estimations and tags in simulation.
      simDebugField = new Field2d();
      SmartDashboard.putData("Vision/SimField", simDebugField);
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
  private void updateEstimationStdDevsForLimelight(Optional<PoseEstimate> estimatedPose) {
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
    for (String name : limelightCameras) {
      LimelightHelpers.SetRobotOrientation(
          name, yawdegrees, 0, 0, 0, 0, 0);

      PoseEstimate pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
      if (pose.tagCount >= 1) {
        curStdDevs = pose.tagCount > 1 ? kMultiTagStdDevs : kSingleTagStdDevs;
        return Optional.of(pose);
      }
    }
    return Optional.empty();
  }

  public Optional<PoseEstimate> getVisionMeasurement_MT1() {
    // Collect pose estimates and per-camera ambiguity
    List<PoseEstimate> candidates = new ArrayList<>();
    List<String> names = new ArrayList<>();

    for (String name : limelightCameras) {
      PoseEstimate pose = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
      if (pose == null) {
        SmartDashboard.putNumber("Vision/Camera/" + name + "/Ambiguity", -1);
        continue;
      }

      double minAmb = Double.MAX_VALUE;
      if (pose.rawFiducials != null && pose.rawFiducials.length > 0) {
        for (var rf : pose.rawFiducials) {
          if (rf == null) continue;
          minAmb = Math.min(minAmb, rf.ambiguity);
        }
      }

      SmartDashboard.putNumber("Vision/Camera/" + name + "/Ambiguity", minAmb == Double.MAX_VALUE ? -1 : minAmb);

      candidates.add(pose);
      names.add(name);
    }

    if (candidates.isEmpty()) {
      updateEstimationStdDevsForLimelight(Optional.empty());
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
      SmartDashboard.putString("Vision/PendingSwitchTo", candidateName);
      SmartDashboard.putNumber("Vision/SwitchConfidence", progress);

      if (switchConfidence >= CONFIRM_THRESHOLD) {
        // commit
        lastChosenPoseEstimate = bestPose;
        lastSelectedCameraName = candidateName;
        lastSelectedAmbiguity = bestAmb;
        switchConfidence = 0.0;
        SmartDashboard.putString("Vision/PendingSwitchTo", "");
        SmartDashboard.putNumber("Vision/SwitchConfidence", 1.0);
      }
    }

    // Update stddevs and return the currently chosen pose estimate
    updateEstimationStdDevsForLimelight(Optional.of(lastChosenPoseEstimate));
    SmartDashboard.putString("Vision/SelectedCamera", lastSelectedCameraName == null ? "" : lastSelectedCameraName);
    SmartDashboard.putNumber("Vision/SelectedAmbiguity", lastSelectedAmbiguity == Double.MAX_VALUE ? -1 : lastSelectedAmbiguity);

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
                  est.pose, Utils.fpgaToCurrentTime(est.timestampSeconds), this.getEstimationStdDevs());
            }
          });
    }
  }

  public Command updateGlobalPoseCommand(CommandSwerveDrivetrain drivetrain) {
    return run(() -> this.updateGlobalPose(drivetrain)).withName("UpdateGlobalPoseCommand");
  }

  // ----- Simulation
  public void simulationPeriodic(Pose2d robotSimPose) {
    if (simDebugField != null) simDebugField.setRobotPose(robotSimPose);
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
    if (Robot.isSimulation() && simDebugField != null) simDebugField.setRobotPose(pose);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    return simDebugField;
  }
}
