package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.List;
import java.util.function.Supplier;

/**
 * Largely written by Eeshwar based off their blog at https://blog.eeshwark.com/robotblog/shooting-on-the-fly
 */
public class ShootOnTheMoveCommand extends Command
{
  private static final double SHOOTER_WHEEL_DIAMETER_METERS = Inches.of(4).in(Meters);

  // Subsystems
  private final ShooterSubsystem   shooterSubsystem;

  /**
   * Current robot pose. (Blue-alliance)
   */
  private final Supplier<Pose2d>        robotPose;
  /**
   * Current field-oriented chassis speeds.
   */
  private final Supplier<ChassisSpeeds> fieldOrientedChassisSpeeds;
  /**
   * Pose to shoot at.
   */
  private final Pose2d                  goalPose;

  /**
   * Time in seconds between when the robot is told to move and when the shooter actually shoots.
   */
  private final double latency = 0.15;
  /**
   * Maps Distance to RPM
   */
  private final InterpolatingDoubleTreeMap shooterTable = new InterpolatingDoubleTreeMap();
  private double targetFlywheelRpm = 0.0;
  private Command flywheelVelocityCommand;

  /**
   * Shoot on the move command to always have the turret ready to fire.
   *
   * @param turret                     Turret subsystem
   * @param hood                       Hood subsystem
   * @param flyWheel                   Flywheel subsystem
   * @param currentPose                Current robot pose.
   * @param fieldOrientedChassisSpeeds Current field-oriented chassis speeds.
   * @param goal                       Goal to shoot at.
   */
  public ShootOnTheMoveCommand(ShooterSubsystem shooter,
                               Supplier<Pose2d> currentPose, Supplier<ChassisSpeeds> fieldOrientedChassisSpeeds,
                               Pose2d goal)
  {
    shooterSubsystem = shooter;
    robotPose = currentPose;
    this.fieldOrientedChassisSpeeds = fieldOrientedChassisSpeeds;
    this.goalPose = goal;

    // Test Results
    for (var entry : List.of(Pair.of(Meters.of(1), RPM.of((1000))),
                             Pair.of(Meters.of(2), RPM.of(2000)),
                             Pair.of(Meters.of(3), RPM.of(3000)))
    )
    {shooterTable.put(entry.getFirst().in(Meters), entry.getSecond().in(RPM));}

    addRequirements(shooterSubsystem.turret);
    setName("Shoot on the move");
  }

  @Override
  public void initialize()
  {
    flywheelVelocityCommand =
        shooterSubsystem.flywheel
            .setVelocity(() -> RPM.of(targetFlywheelRpm))
            .withName("ShootOnTheMoveFlywheelVelocity");
    CommandScheduler.getInstance().schedule(flywheelVelocityCommand);
  }

  @Override
  public void execute()
  {
    // Please look here for the original authors work!
    // https://blog.eeshwark.com/robotblog/shooting-on-the-fly
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // YASS did not come up with this
    // -------------------------------------------------------

    var robotSpeed = fieldOrientedChassisSpeeds.get();
    // 1. LATENCY COMP
    Translation2d futurePos = robotPose.get().getTranslation().plus(
        new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond).times(latency)
                                                                   );

    // 2. GET TARGET VECTOR
    Translation2d goalLocation = goalPose.getTranslation();
    Translation2d targetVec    = goalLocation.minus(futurePos);
    double        dist         = targetVec.getNorm();

    // 3. CALCULATE IDEAL SHOT (Stationary)
    // Convert tuned flywheel RPM values to equivalent wheel-edge linear speed.
    double idealHorizontalSpeed = rpmToLinearSpeedMetersPerSecond(shooterTable.get(dist));

    // 4. VECTOR SUBTRACTION
    Translation2d robotVelVec = new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
    Translation2d shotVec     = targetVec.div(dist).times(idealHorizontalSpeed).minus(robotVelVec);

    // 5. CONVERT TO CONTROLS
    double turretAngle = shotVec.getAngle().getDegrees();
    double newHorizontalSpeed = shotVec.getNorm();
    targetFlywheelRpm = linearSpeedToRpm(newHorizontalSpeed);

    // 7. SET OUTPUTS
    shooterSubsystem.turret.setAngleDirect(Degrees.of(turretAngle));
  }

  @Override
  public boolean isFinished()
  {
    // This command is intended to be run until cancelled (for example, during
    // a teleop period where you always want the turret tracking).  Higher-level
    // commands or button bindings can cancel it when shooting is no longer
    // required.
    return false;
  }

  @Override
  public void end(boolean interrupted)
  {
    if (flywheelVelocityCommand != null) {
      flywheelVelocityCommand.cancel();
      flywheelVelocityCommand = null;
    }
  }

  private static double rpmToLinearSpeedMetersPerSecond(double rpm) {
    return (rpm / 60.0) * (Math.PI * SHOOTER_WHEEL_DIAMETER_METERS);
  }

  private static double linearSpeedToRpm(double linearSpeedMetersPerSecond) {
    return (linearSpeedMetersPerSecond / (Math.PI * SHOOTER_WHEEL_DIAMETER_METERS)) * 60.0;
  }
}
