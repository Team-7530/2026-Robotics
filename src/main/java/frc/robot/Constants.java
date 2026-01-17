package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import frc.robot.generated.TunerConstants;
import java.util.List;

public final class Constants {

  public static final boolean USE_POSITIONCONTROL = false;

  public static final double STICK_DEADBAND = 0.02;
  public static final double TRIGGER_SPEEDFACTOR = 0.5;
  public static final double POSITION_TOLERANCE = 0.05;

  public static final class Vision {

    public static final boolean USE_LIMELIGHT = true;
    public static final String LIMELIGHTNAME = "limelight";
  public static final String LIMELIGHTURL = "limelight.local";
  // Optional second limelight (e.g. limelight2). Update name/URL if you have a second camera.
  public static final String LIMELIGHTNAME_2 = "limelight-two";
  public static final String LIMELIGHTURL_2 = "limelight-two.local";
    // PhotonVision is no longer used; keep vendordeps if needed but remove runtime URL constant.

    // Cam mounted - x = +toward front, 0 center, -toward rear in meters.
    //               y = +left of center, 0 center, -right of center in meters
    //               z = +up from base of robot in meters
    //              roll = rotate around front/rear in radians. PI = upsidedown
    //              pitch = tilt down/up along left/right axis. PI/4 = tilt down 45 degrees, -PI/4 =
    // tilt up 45
    //              yaw = rotate left/right around z axis. PI/4 = rotate camera to the left 45
    // degrees.
    public static final List<Pair<String, Transform3d>> kCamerasList =
        List.of(
            // Pair.of("OV9281", new Transform3d(new Translation3d(0.28, 0, 0.15), new Rotation3d(0,
            // 0, 0))),
            // Pair.of("OV9281-2", new Transform3d(new Translation3d(0.228, -0.3048, 0.16), new
            // Rotation3d(0, 0, 0))),
          Pair.of(
                LIMELIGHTNAME,
                new Transform3d(new Translation3d(0.28, 0, 0.16), new Rotation3d(0, 0, 0))),
              // Example second limelight mounted slightly left (+y) of center. Adjust transform to match
              // your physical mounting.
              Pair.of(
                LIMELIGHTNAME_2,
                new Transform3d(new Translation3d(0.28, 0.10, 0.16), new Rotation3d(0, 0, 0))));

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
  }

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

  public static final class ScoringConstants {
    public static final double L1ArmPosition = 0.28;
    public static final double L1RakePosition = -0.3;

    public static final double L2ArmPosition = 0.25;
    public static final double L2RakePosition = -0.22;

    public static final double LoadArmPosition = 0.25;
    public static final double LoadRakePosition = -0.125;

    public static final double ClimbArmPosition = 0.405;
    public static final double ClimbRakePosition = 0.20;

    public static final double CruiseArmPosition = 0.405;
    public static final double CruiseRakePosition = -0.33; //

    public static final Translation2d L2BackupAmount = new Translation2d(-0.3, 0.0);
  }

  public static final class ArmConstants {
    public static final CANBus CANBUS = new CANBus("CANFD");
    public static final int ARMMOTOR_ID = 31;
    public static final int ARMENCODER_ID = 32;

    public static final InvertedValue kArmInverted = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue kArmNeutralMode = NeutralModeValue.Brake;
    public static final SensorDirectionValue kArmEncoderDirection =
        SensorDirectionValue.CounterClockwise_Positive;
    public static final double kArmEncoderOffset = -0.37;

    public static final double kArmChainRatio = 74.0 / 50.0;
    public static final double kArmGearboxRatio = 64.0; // 1:64
    public static final double kArmGearRatio =
        kArmChainRatio * kArmGearboxRatio; // chain ratio * Gearbox ratio

    public static final double armMotorKG = 0.5;
    public static final double armMotorKS = 0.0;
    public static final double armMotorKV = 0.0;
    public static final double armMotorKA = 0.0;
    public static final double armMotorKP = 30.0; // 45
    public static final double armMotorKI = 0.0;
    public static final double armMotorKD = 0.0;
    public static final double MMagicCruiseVelocity = 1;
    public static final double MMagicAcceleration = 2;
    public static final double MMagicJerk = 8000;
    public static final double MMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
    public static final double MMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)
    public static final double peakForwardVoltage = 8.0; // Peak output of 8 volts
    public static final double peakReverseVoltage = -8.0; // Peak output of 8 volts

    public static final double kArmPositionMax = 0.405;
    public static final double kArmPositionMin = 0.138;

    public static final double kTargetArmHigh = 0.405;
    public static final double kTargetArmLow = 0.138;
    public static final double kArmTeleopSpeed = 0.1;
    public static final double kArmTeleopFactor = 0.02;
  }

  public static final class ShooterConstants {
    public static final CANBus CANBUS = new CANBus("CANFD");

    // Placeholder CAN IDs - update to match your wiring
    public static final int TURRET_MASTER_ID = 51;
    public static final int TURRET_ENCODER_ID = 52;

    public static final int FLYWHEEL_MASTER_ID = 53;
    public static final int FLYWHEEL_FOLLOWER_ID = 54;

    // Turret limits in degrees (180-degree travel centered on 0)
    public static final double TURRET_MIN_DEG = -90.0;
    public static final double TURRET_MAX_DEG = 90.0;

    public static final InvertedValue kTurretInverted = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue kTurretNeutralMode = NeutralModeValue.Brake;
    public static final SensorDirectionValue kTurretEncoderDirection =
        SensorDirectionValue.CounterClockwise_Positive;
    public static final double kTurretEncoderOffset = 0.0;

    public static final double kTurretChainRatio = 1.0 / 1.0;
    public static final double kTurretGearboxRatio = 1.0; // 1:1
    // Gear ratio between rotor and turret (1.0 means 1 rotor rev = 360 deg)
    public static final double kTurretGearRatio =
        kTurretChainRatio * kTurretGearboxRatio; // chain ratio * Gearbox ratio

    public static final double turretMotorKS = 0.0;
    public static final double turretMotorKV = 0.0;
    public static final double turretMotorKA = 0.0;
    public static final double turretMotorKP = 30.0; // 45
    public static final double turretMotorKI = 0.0;
    public static final double turretMotorKD = 0.0;
    public static final double MMagicCruiseVelocity = 1;
    public static final double MMagicAcceleration = 2;
    public static final double MMagicJerk = 8000;
    public static final double MMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
    public static final double MMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)

    public static final double kTurretPeakForwardVoltage = 8.0; // Peak output of 8 volts
    public static final double kTurretPeakReverseVoltage = -8.0; // Peak output of 8 volts

    // Flywheel tuning
    public static final double FLYWHEEL_MAX_RPM = 6000.0; // adjust to your flywheel
    public static final double FLYWHEEL_DEFAULT_RPM = 3000.0;

    public static final InvertedValue kFlywheelInverted = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue kFlywheelNeutralMode = NeutralModeValue.Brake;

    public static final double kFlywheelChainRatio = 1.0 / 1.0;
    public static final double kFlywheelGearboxRatio = 1.0; // 1:1
    // Gear ratio between rotor and turret (1.0 means 1 rotor rev = 360 deg)
    public static final double kFlywheelGearRatio =
        kFlywheelChainRatio * kFlywheelGearboxRatio; // chain ratio * Gearbox ratio

    // Flywheel control tuning (Velocity closed-loop)
    public static final double FLYWHEEL_kS = 0.0;
    public static final double FLYWHEEL_kP = 0.1;
    public static final double FLYWHEEL_kI = 0.0;
    public static final double FLYWHEEL_kD = 0.0;

    public static final double kFlywheelPeakForwardVoltage = 8.0; // Peak output of 8 volts
    public static final double kFlywheelPeakReverseVoltage = -8.0; // Peak output of 8 volts

    public static final double kTurretTeleopSpeed = 0;

  }

  public static final class RakeConstants {
    public static final CANBus CANBUS = CANBus.roboRIO();
    public static final int RAKEMOTOR_ID = 33;
    public static final int RAKEENCODER_ID = 34;

    public static final InvertedValue kRakeInverted = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue kRakeNeutralMode = NeutralModeValue.Brake;
    public static final SensorDirectionValue kRakeEncoderDirection =
        SensorDirectionValue.Clockwise_Positive;
    public static final double kRakeEncoderOffset = -0.171;// add 0.25 offset, sub it later

    public static final double kRakeChainRatio = 1.0; // 1:1
    public static final double kRakeGearboxRatio = 45.0; // 1:45
    public static final double kRakeGearRatio =
        kRakeChainRatio * kRakeGearboxRatio; // chain ratio * Gearbox ratio

    public static final double rakeMotorKG = 0.0;
    public static final double rakeMotorKS = 0.0;
    public static final double rakeMotorKS_slow = 0.0;
    public static final double rakeMotorKV = 0.0;
    public static final double rakeMotorKA = 0.0;
    public static final double rakeMotorKP = 35.0; // 70
    public static final double rakeMotorKP_slow = 25.0;
    public static final double rakeMotorKI = 0.0;
    public static final double rakeMotorKD = 0.0;
    public static final double MMagicCruiseVelocity = 0;
    public static final double MMagicAcceleration = 0;
    public static final double MMagicJerk = 0;
    public static final double MMagicExpo_kV = 4.8; // kV is around 0.12 V/rps
    public static final double MMagicExpo_kA = 4.8; // Use a slower kA of 0.1 V/(rps/s)
    public static final double peakForwardVoltage = 8.0; // Peak output of 8 volts
    public static final double peakReverseVoltage = -8.0; // Peak output of 8 volts

    public static final double kRakePositionMax = 0.25;
    public static final double kRakePositionMin = -0.334;

    public static final double kRakeTeleopSpeed = 0.1;
    public static final double kRakeTeleopFactor = 0.05;
  }

  public static final class IntakeConstants {
    public static final CANBus CANBUS = CANBus.roboRIO();
    public static final int LINTAKEMOTOR_ID = 35;
    public static final int RINTAKEMOTOR_ID = 36;
    public static final int RANGESENSOR_ID = 37;

    public static final InvertedValue kLIntakeInverted = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue kRIntakeInverted = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue kIntakeNeutralMode = NeutralModeValue.Coast;
    public static final double peakForwardVoltage = 10.0; // Peak output of 8 volts
    public static final double peakReverseVoltage = -10.0; // Peak output of 8 volts
    public static final double peakForwardTorqueCurrent = 40.0; // Peak output of 40 amps
    public static final double peakReverseTorqueCurrent = -40.0; // Peak output of 40 amps

    public static final double kIntakeChainRatio = 24.0 / 10.0; // 24:10
    public static final double kIntakeGearboxRatio = 1.0; // 1:1
    public static final double kIntakeGearRatio = kIntakeChainRatio * kIntakeGearboxRatio;

    /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    public static final double intakeMotorTorqueKS = 0.0; // Static feedforward gain
    public static final double intakeMotorTorqueKP = 8.0; // error of 1 rps results in 8 amps output
    public static final double intakeMotorTorqueKI = 0.2; // error of 1 rps incr by 0.2 amps per sec
    public static final double intakeMotorTorqueKD = 0.001; // 1000 rps^2 incr 1 amp output

    public static final double intakeVelocity = -3.0;
    public static final double outtakeL1VelocityL = 2.0;
    public static final double outtakeL1VelocityR = 5.0;
    public static final double outtakeL2Velocity = 8.0;

    public static final Distance rangeThreshold = Inches.of(5.0);
    public static final double kRangeFOVCenterX = 0;
    public static final double kRangeFOVCenterY = 0;
    public static final double kRangeFOVRangeX = 27;
    public static final double kRangeFOVRangeY = 27;
    public static final double kProxThreshold = 0.1;
    public static final double kProxHysteresis = 0.01;
    public static final double kMinSigStrength = 2500;
  }

  public static final class FeederConstants{
    public static final CANBus CANBUS = CANBus.roboRIO();
    public static final int FEEDERMOTOR_ID = 65;
    public static final int RANGESENSOR_ID = 67;

    public static final InvertedValue kFeederInverted = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue kFeederNeutralMode = NeutralModeValue.Coast;
    public static final double peakForwardVoltage = 10.0; // Peak output of 8 volts
    public static final double peakReverseVoltage = -10.0; // Peak output of 8 volts
    public static final double peakForwardTorqueCurrent = 40.0; // Peak output of 40 amps
    public static final double peakReverseTorqueCurrent = -40.0; // Peak output of 40 amps

    public static final double kFeederChainRatio = 24.0 / 10.0; // 24:10
    public static final double kFeederGearboxRatio = 1.0; // 1:1
    public static final double kFeederGearRatio = kFeederChainRatio * kFeederGearboxRatio;

    /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    public static final double feederMotorTorqueKS = 0.0; // Static feedforward gain
    public static final double feederMotorTorqueKP = 8.0; // error of 1 rps results in 8 amps output
    public static final double feederMotorTorqueKI = 0.2; // error of 1 rps incr by 0.2 amps per sec
    public static final double feederMotorTorqueKD = 0.001; // 1000 rps^2 incr 1 amp output

    public static final double feederVelocity = -3.0;

    public static final Distance rangeThreshold = Inches.of(5.0);
    public static final double kRangeFOVCenterX = 0;
    public static final double kRangeFOVCenterY = 0;
    public static final double kRangeFOVRangeX = 27;
    public static final double kRangeFOVRangeY = 27;
    public static final double kProxThreshold = 0.1;
    public static final double kProxHysteresis = 0.01;
    public static final double kMinSigStrength = 2500;
  }

  public static final class ClimberConstants {
    public static final CANBus CANBUS = new CANBus("CANFD");
    public static final int CLIMBMOTOR_ID = 41;
    public static final int CLIMBMOTORFOLLOWER_ID = 42;
    public static final int CLIMBENCODER_ID = 43;
    public static final int CLAMPSERVO_ID = 0; // Rachet servo 1 plugged into PWM 0
    public static final int CLAMPSERVOFOLLOWER_ID = 1; // Rachet servo 2 plugged into PWM 1

    public static final InvertedValue kClimberInverted = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue kClimberFollowerInverted = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue kClimberNeutralMode = NeutralModeValue.Brake;
    public static final SensorDirectionValue kClimberEncoderDirection =
        SensorDirectionValue.CounterClockwise_Positive;

    public static final double kClimberChainRatio = 28.0 / 10.0;
    public static final double kClimberGearboxRatio = 48.0; // 1:48
    public static final double kClimberGearRatio =
        kClimberChainRatio * kClimberGearboxRatio; // chain ratio * Gearbox ratio

    public static final double kClimberEncoderOffset = 0.06275;
    public static final double kClimberEncoderMin = 0.0;
    public static final double kClimberEncoderMax = 0.45;

    public static final double peakForwardVoltage = 10.0; // Peak output of 10 volts
    public static final double peakReverseVoltage = -8.0; // Peak output of 10 volts
    public static final double peakForwardTorqueCurrent = 120.0; // Peak output of 80 amps
    public static final double peakReverseTorqueCurrent = -80.0; // Peak output of 80 amps

    public static final double climbMotorKG = 0.0;
    public static final double climbMotorKS = 0.0;
    public static final double climbMotorKV = 0.0;
    public static final double climbMotorKA = 0.0;
    public static final double climbMotorKP = 60.0;
    public static final double climbMotorKI = 0.0;
    public static final double climbMotorKD = 0.0;

    public static final double climbMotorTorqueKP = 240.0; // 0.25 rot err == 60 A output
    public static final double climbMotorTorqueKI = 0.0; // No output for integrated error
    public static final double climbMotorTorqueKD = 6.0; // vel of 1 rps == 6 A output

    public static final double MMagicCruiseVelocity = 2;
    public static final double MMagicAcceleration = 1;
    public static final double MMagicJerk = 0;

    public static final double kUnclampedPositionFollower = 0.46;
    public static final double kClampedPositionFollower = 0.78;

    public static final double kUnclampedPosition = 0.65; // Min and Max opposite of Original
    public static final double kClampedPosition = 0.35;

    public static final double kClimberPositionMin = kClimberEncoderMin;
    public static final double kClimberPositionMax = kClimberEncoderMax;

    public static final double kTargetClimberStart = 0.04;
    public static final double kTargetClimberFull = 0.26;

    public static final double kClimberSpeed = 0.8;
    public static final double kClimbTeleopFactor = 10.0;
  }
}
