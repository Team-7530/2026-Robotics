package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
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

@Logged
public class Telemetry {
  /**
   * The maximum speed the drivetrain is currently allowed to achieve.
   * Changes during the match (cruise/slow/max modes) are critical diagnostics.
   */
  @Logged(importance = Logged.Importance.INFO)
  private double m_maxSpeed;
  /** if true, debug values (marked by callers) will be published; otherwise
   * they are suppressed to save bandwidth/CPU during competition runs. */
  @NotLogged
  private final boolean m_debugMode;

  /* What to publish over networktables for telemetry */
  @NotLogged
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /* Robot pose visualization */
  @Logged(importance = Logged.Importance.CRITICAL)
  private final Field2d field = new Field2d();

  /* Shuffleboard tab and cached entries for fewer allocations */
  @NotLogged
  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Telemetry");
  @NotLogged
  private final Map<String, GenericEntry> m_entries = new HashMap<>();

  /* Robot speeds for general checking */
  @NotLogged
  private final NetworkTable driveStats = inst.getTable("Drive");
  private final DoublePublisher velocityX = driveStats.getDoubleTopic("Velocity X").publish();
  private final DoublePublisher velocityY = driveStats.getDoubleTopic("Velocity Y").publish();
  private final DoublePublisher speed = driveStats.getDoubleTopic("Speed").publish();
  private final DoublePublisher odomFreq = driveStats.getDoubleTopic("Odometry Frequency").publish();

  /* Keep a reference of the last pose to calculate the speeds */
  @Logged(importance = Logged.Importance.DEBUG)
  private Pose2d m_lastPose = new Pose2d();
  @Logged(importance = Logged.Importance.DEBUG)
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

  @Logged(importance = Logged.Importance.CRITICAL)
  private double m_poseX = 0.0;
  @Logged(importance = Logged.Importance.CRITICAL)
  private double m_poseY = 0.0;
  @Logged(importance = Logged.Importance.CRITICAL)
  private double m_poseRotation = 0.0;
  @Logged(importance = Logged.Importance.CRITICAL)
  private double m_odometryPeriod = 0.0;

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
    m_maxSpeed = maxSpeed;
    m_debugMode = debugMode;  

    // SignalLogger.start();
    SignalLogger.stop();
    SignalLogger.enableAutoLogging(false);
    
    // register the common sendables once on the shuffleboard tab
    putData("Field", field);
    for (int i = 0; i < m_moduleMechanisms.length; ++i) {
      putData("Module " + i, m_moduleMechanisms[i]);
    }
  }

  /** Update the configured drivetrain max speed in m/s for telemetry scaling and display. */
  public void setMaxSpeed(double maxSpeed) {
    m_maxSpeed = maxSpeed;
  }

  /* Convenience helpers so subsystems stop calling SmartDashboard directly */
  public void putData(String key, Sendable sendable) {
    m_tab.add(key, sendable);
  }

  public void putNumber(String key, double value) {
    if (Double.isFinite(value)) {
      GenericEntry entry = m_entries.computeIfAbsent(key, k -> m_tab.add(k, value).getEntry());
      entry.setDouble(value);
    }
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

  /** Accept the swerve drive state and telemeterize it to the dashboard (Shuffleboard) and SignalLogger. */
  public void telemeterize(SwerveDriveState state) {
    /* Telemeterize the pose */
    var pose = state.Pose;
    m_poseX = pose.getX();
    m_poseY = pose.getY();
    m_poseRotation = pose.getRotation().getDegrees();
    m_odometryPeriod = state.OdometryPeriod;

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
    if (m_odometryPeriod > 1e-6) {
      odomFreq.set(1.0 / m_odometryPeriod);
    }

    // also publish the drivetrain maximum speed; this is used on the
    // competition layout so drivers know what the robot is currently allowed to do.
    putNumber("DriveTrain/MaxSpeed", m_maxSpeed, true);
    putNumber("DriveTrain/PoseX", m_poseX, true);
    putNumber("DriveTrain/PoseY", m_poseY, true);
    putNumber("DriveTrain/PoseTheta", m_poseRotation, true);

    /* Telemeterize the module's states */
    double moduleSpeedDenominator = Math.max(1e-6, 2.0 * m_maxSpeed);
    for (int i = 0; i < 4; ++i) {
      m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
      m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
      m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / moduleSpeedDenominator);
    }

    // SignalLogger.writeDoubleArray("odometry", m_poseArray);
    // SignalLogger.writeDouble("odom period", m_odometryPeriod, "seconds");
  }
}
