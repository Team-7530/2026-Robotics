/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.Vision.*;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.limelightvision.LimelightHelpers;
import frc.lib.limelightvision.LimelightHelpers.PoseEstimate;
import frc.lib.util.FieldConstants;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem implements Subsystem {
  private final List<Pair<PhotonCamera, PhotonPoseEstimator>> photonCameras = new ArrayList<>();
  private final List<String> limelightCameras = new ArrayList<>();
  private Matrix<N3, N1> curStdDevs;

  // Simulation
  private final List<PhotonCameraSim> photonCameraSims = new ArrayList<>();
  private VisionSystemSim visionSim;

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
        PhotonPoseEstimator photonEstimator =
            new PhotonPoseEstimator(
                FieldConstants.fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, pose);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        photonCameras.add(Pair.of(new PhotonCamera(name), photonEstimator));
      }
    }

    if (Robot.isReal()) {
      cam0 = CameraServer.startAutomaticCapture();
      cam0.setResolution(240, 160);
      cam0.setFPS(15);
    }

    // ----- Simulation
    if (Robot.isSimulation()) {
      // Create the vision system simulation which handles cameras and targets on the field.
      visionSim = new VisionSystemSim("main");
      // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
      visionSim.addAprilTags(FieldConstants.fieldLayout);
      // Create simulated camera properties. These can be set to mimic your actual camera.
      var cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(90));
      cameraProp.setCalibError(0.25, 0.10);
      cameraProp.setFPS(60);
      cameraProp.setAvgLatencyMs(10);
      cameraProp.setLatencyStdDevMs(10);

      for (var camera : photonCameras) {
        PhotonCameraSim cameraSim = new PhotonCameraSim(camera.getFirst(), cameraProp);
        // Add the simulated camera to view the targets on this simulated field.
        visionSim.addCamera(cameraSim, camera.getSecond().getRobotToCameraTransform());
        cameraSim.enableDrawWireframe(true);
        photonCameraSims.add(cameraSim);
      }
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
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    List<PhotonTrackedTarget> allCameraTargets = new ArrayList<>();
    List<EstimatedRobotPose> allVisionEstimates = new ArrayList<>();

    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var camera : photonCameras) {
      for (var change : camera.getFirst().getAllUnreadResults()) {
        allCameraTargets.addAll(change.getTargets());

        visionEst = camera.getSecond().update(change);
        if (visionEst.isPresent()) {
          allVisionEstimates.add(visionEst.get());
        }

        if (Robot.isSimulation()) {
          visionEst.ifPresentOrElse(
              est ->
                  getSimDebugField()
                      .getObject("VisionEstimation" + camera.getFirst().getName())
                      .setPose(est.estimatedPose.toPose2d()),
              () ->
                  getSimDebugField()
                      .getObject("VisionEstimation" + camera.getFirst().getName())
                      .setPoses());
        }
      }
    }
    updateEstimationStdDevs(
        allVisionEstimates.isEmpty()
            ? Optional.empty()
            : Optional.of(allVisionEstimates.get(allVisionEstimates.size() - 1)),
        allCameraTargets,
        photonCameras.get(0).getSecond());

    return averageVisionEstimates(allVisionEstimates);
  }

  private Optional<EstimatedRobotPose> averageVisionEstimates(List<EstimatedRobotPose> estimates) {
    if (estimates.isEmpty()) {
      return Optional.empty();
    }

    double x = 0.0, y = 0.0, rotation = 0.0;
    double latestTimestamp = 0.0;
    List<PhotonTrackedTarget> combinedTargets = new ArrayList<>();
    Set<Integer> seenTargets = new HashSet<>();

    for (EstimatedRobotPose estimate : estimates) {
      Pose2d pose = estimate.estimatedPose.toPose2d();
      x += pose.getX();
      y += pose.getY();
      rotation += pose.getRotation().getRadians();

      if (estimate.timestampSeconds > latestTimestamp) {
        latestTimestamp = estimate.timestampSeconds;
      }

      for (PhotonTrackedTarget target : estimate.targetsUsed) {
        if (seenTargets.add(target.getFiducialId())) {
          combinedTargets.add(target);
        }
      }
    }

    x /= estimates.size();
    y /= estimates.size();
    rotation /= estimates.size();

    Pose3d avgPose = new Pose3d(x, y, 0, new Rotation3d(0, 0, rotation));

    return Optional.of(
        new EstimatedRobotPose(
            avgPose, latestTimestamp, combinedTargets, estimates.get(0).strategy));
  }

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets All targets in this camera frame
   */
  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose,
      List<PhotonTrackedTarget> targets,
      PhotonPoseEstimator estimator) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
        var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) continue;
        numTags++;
        avgDist +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        curStdDevs = kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curStdDevs = estStdDevs;
      }
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
    for (String name : limelightCameras) {
      PoseEstimate pose = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
      if (pose.tagCount > 1) {
        curStdDevs = kMultiTagStdDevs;
        return Optional.of(pose);
      } else if ((pose.tagCount == 1)
              && (pose.rawFiducials.length == 1)
              && (pose.rawFiducials[0].ambiguity <= 0.7)
              && (pose.rawFiducials[0].distToCamera <= 3.0)) {
        curStdDevs = kSingleTagStdDevs;
        return Optional.of(pose);
      }
    }
    return Optional.empty();
  }

  public void updateGlobalPose(CommandSwerveDrivetrain drivetrain) {
    if ((RobotState.isAutonomous() || RobotState.isTest()) &&
        (Math.abs(drivetrain.getState().Speeds.vxMetersPerSecond) < 0.2) &&
        (Math.abs(drivetrain.getState().Speeds.vyMetersPerSecond) < 0.2) &&
        (Math.abs(drivetrain.getState().Speeds.omegaRadiansPerSecond) < RotationsPerSecond.of(2).in(RadiansPerSecond))) {

      if (!USE_LIMELIGHT) {
        // Correct pose estimate with vision measurements
        var visionEst = this.getEstimatedGlobalPose();
        visionEst.ifPresent(
            est -> {
              drivetrain.addVisionMeasurement(
                  est.estimatedPose.toPose2d(),
                  Utils.fpgaToCurrentTime(est.timestampSeconds),
                  this.getEstimationStdDevs());
            });
      } else {
        // var limelightEst1 = this.getVisionMeasurement_MT2(drivetrain.getPigeon2().getYaw().getValue().in(Degrees));
        var limelightEst = this.getVisionMeasurement_MT1();
        limelightEst.ifPresent(
            est -> {
              if (est.tagCount >= 1) {
                {
                  drivetrain.addVisionMeasurement(
                      est.pose,
                      Utils.fpgaToCurrentTime(est.timestampSeconds),
                      this.getEstimationStdDevs());
                }
              }
            });
      }
    }
  }

  public Command updateGlobalPoseCommand(CommandSwerveDrivetrain drivetrain) {
    return run(() -> this.updateGlobalPose(drivetrain)).withName("UpdateGlobalPoseCommand");
  }

  // ----- Simulation
  public void simulationPeriodic(Pose2d robotSimPose) {
    visionSim.update(robotSimPose);
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
    if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) return null;
    return visionSim.getDebugField();
  }
}
