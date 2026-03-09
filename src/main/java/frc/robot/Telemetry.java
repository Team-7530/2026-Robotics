package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import java.util.HashMap;
import java.util.Map;

public class Telemetry {
  private final double MaxSpeed;
  /** if true, debug values (marked by callers) will be published; otherwise
   * they are suppressed to save bandwidth/CPU during competition runs. */
  private final boolean m_debugMode;

  /**
   * Construct a telemetry object, with the specified max speed of the robot
   *
   * @param maxSpeed Maximum speed in meters per second
   */
  public Telemetry(double maxSpeed) {
    this(maxSpeed, false);
  }

  /**
   * @param maxSpeed maximum drivetrain speed (m/s); also telemeterized
   * @param debugMode whether to emit debug telemetry entries
   */
  public Telemetry(double maxSpeed, boolean debugMode) {
    MaxSpeed = maxSpeed;
    m_debugMode = debugMode;
    SignalLogger.start();
    // SignalLogger.stop();
    // register the common sendables once on the shuffleboard tab
    putData("Field", field);
    for (int i = 0; i < m_moduleMechanisms.length; ++i) {
      putData("Module " + i, m_moduleMechanisms[i]);
    }
  }

  /**
   * Enable or disable emission of parameters marked as debug.  During a
   * match this should normally be left false to reduce CPU/NT traffic; in
   * test or tuning modes you can flip it on.
   */
  public void setDebugMode(boolean debug) {
    // m_debugMode is final, so we can't reassign it; in practice callers
    // should choose the correct value at construction time.  If runtime
    // toggling is required we could remove the final modifier and update
    // accordingly, but for now we just document that limitation.
    //
    // For example, you might create the telemetry object with
    //   new Telemetry(maxSpeed, DriverStation.isTest())
    // or read a dashboard switch and rebuild the object.
    //
    // This method exists as a no-op placeholder to keep the API stable.
  }

  /* Convenience helpers so subsystems stop calling SmartDashboard directly */
  public void putData(String key, Sendable sendable) {
    m_tab.add(key, sendable);
  }

  public void putNumber(String key, double value) {
    GenericEntry entry = m_entries.computeIfAbsent(key, k -> m_tab.add(k, value).getEntry());
    entry.setDouble(value);
  }

  /**
   * Like {@link #putNumber(String,double)} but only sends the value if
   * {@link #m_debugMode} is true.  Use this for expensive or verbose data that
   * you only care about during tuning/diagnostics.
   */
  public void putNumber(String key, double value, boolean alwaysLog) {
    if (m_debugMode || alwaysLog)
      putNumber(key, value);
  }

  public void putString(String key, String value) {
    GenericEntry entry = m_entries.computeIfAbsent(key, k -> m_tab.add(k, value).getEntry());
    entry.setString(value);
  }

  public void putString(String key, String value, boolean alwaysLog) {
    if (m_debugMode || alwaysLog)
      putString(key, value);
  }

  /** Publish a simple boolean value under the telemetry tab. */
  public void putBoolean(String key, boolean value) {
    GenericEntry entry = m_entries.computeIfAbsent(key, k -> m_tab.add(k, value).getEntry());
    entry.setBoolean(value);
  }

  public void putBoolean(String key, boolean value, boolean alwaysLog) {
    if (m_debugMode || alwaysLog)
      putBoolean(key, value);
  }

  /* What to publish over networktables for telemetry */
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /* Robot pose visualization */
  private final Field2d field = new Field2d();

  /* Shuffleboard tab and cached entries for fewer allocations */
  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Telemetry");
  private final Map<String, GenericEntry> m_entries = new HashMap<>();

  /* Robot speeds for general checking */
  private final NetworkTable driveStats = inst.getTable("Drive");
  private final DoublePublisher velocityX = driveStats.getDoubleTopic("Velocity X").publish();
  private final DoublePublisher velocityY = driveStats.getDoubleTopic("Velocity Y").publish();
  private final DoublePublisher speed = driveStats.getDoubleTopic("Speed").publish();
  private final DoublePublisher odomFreq =
      driveStats.getDoubleTopic("Odometry Frequency").publish();

  /* Keep a reference of the last pose to calculate the speeds */
  private Pose2d m_lastPose = new Pose2d();
  private double lastTime = Utils.getCurrentTimeSeconds();

  /* Mechanisms to represent the swerve module states */
  private final Mechanism2d[] m_moduleMechanisms =
      new Mechanism2d[] {
        new Mechanism2d(1, 1), new Mechanism2d(1, 1), new Mechanism2d(1, 1), new Mechanism2d(1, 1),
      };
  /* A direction and length changing ligament for speed representation */
  private final MechanismLigament2d[] m_moduleSpeeds =
      new MechanismLigament2d[] {
        m_moduleMechanisms[0]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
      };
  /* A direction changing and length constant ligament for module direction */
  private final MechanismLigament2d[] m_moduleDirections =
      new MechanismLigament2d[] {
        m_moduleMechanisms[0]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
      };

  private final double[] m_poseArray = new double[3];

  /** Accept the swerve drive state and telemeterize it to the dashboard (Shuffleboard) and SignalLogger. */
  public void telemeterize(SwerveDriveState state) {
    /* Telemeterize the pose */
    var pose = state.Pose;
    m_poseArray[0] = pose.getX();
    m_poseArray[1] = pose.getY();
    m_poseArray[2] = pose.getRotation().getDegrees();

    field.setRobotPose(pose);

    /* Telemeterize the robot's general speeds */
    var currentTime = Utils.getCurrentTimeSeconds();
    var diffTime = currentTime - lastTime;
    lastTime = currentTime;

    var distanceDiff = pose.minus(m_lastPose).getTranslation();
    m_lastPose = pose;

    var velocities = diffTime > 1e-6 ? distanceDiff.div(diffTime) : new Translation2d();

    speed.set(velocities.getNorm());
    velocityX.set(velocities.getX());
    velocityY.set(velocities.getY());
    if (state.OdometryPeriod > 1e-6) {
      odomFreq.set(1.0 / state.OdometryPeriod);
    }

    // also publish the drivetrain maximum speed; this is used on the
    // competition layout so drivers know what the robot is currently allowed to do.
    putNumber("DriveTrain/MaxSpeed", MaxSpeed);
    putNumber("DriveTrain/PoseX", m_poseArray[0]);
    putNumber("DriveTrain/PoseY", m_poseArray[1]);
    putNumber("DriveTrain/PoseTheta", m_poseArray[2]);

    /* Telemeterize the module's states */
    for (int i = 0; i < 4; ++i) {
      m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
      m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
      m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));
    }

    SignalLogger.writeDoubleArray("odometry", m_poseArray);
    SignalLogger.writeDouble("odom period", state.OdometryPeriod, "seconds");
  }
}
