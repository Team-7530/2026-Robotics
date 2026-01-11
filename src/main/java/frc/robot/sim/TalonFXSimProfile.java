package frc.robot.sim;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.sim.PhysicsSim.SimProfile;

/** Holds information about a simulated TalonFX. */
class TalonFXSimProfile extends SimProfile {
  private static final double kMotorResistance =
      0.002; // Assume 2mOhm resistance for voltage drop calculation
  private final TalonFXSimState _talonFXSim;
  private final CANcoderSimState _canCoderSim;
  private final double _gearRatio;
  private final boolean _canCoderInvert;

  private final DCMotorSim _motorSim;

  /**
   * Creates a new simulation profile for a TalonFX device.
   *
   * @param talonFX The TalonFX device
   * @param canCoder The CANcoder associated with the TalonFX
   * @param gearRatio The gear ratio from the TalonFX to the mechanism
   * @param rotorInertia Rotational Inertia of the mechanism at the rotor
   */
  public TalonFXSimProfile(
      final TalonFX talonFX,
      final CANcoder canCoder,
      final double gearRatio,
      final double rotorInertia) {
    TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();
    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();

    talonFX.getConfigurator().refresh(talonFXConfig);
    canCoder.getConfigurator().refresh(canCoderConfig);

    this._talonFXSim = talonFX.getSimState();
    this._canCoderSim = canCoder.getSimState();
    this._gearRatio = gearRatio;
    this._canCoderInvert =
        talonFXConfig.MotorOutput.Inverted.value
            != canCoderConfig.MagnetSensor.SensorDirection.value;
    var gearbox = DCMotor.getKrakenX60Foc(1);
    this._motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearbox, rotorInertia, gearRatio), gearbox);
  }

  /**
   * Runs the simulation profile.
   *
   * <p>This uses very rudimentary physics simulation and exists to allow users to test features of
   * our products in simulation using our examples out of the box. Users may modify this to utilize
   * more accurate physics simulation.
   */
  public void run() {
    /// DEVICE SPEED SIMULATION

    _motorSim.setInputVoltage(_talonFXSim.getMotorVoltage());

    _motorSim.update(getPeriod());

    /// SET SIM PHYSICS INPUTS
    final double position_rot = _motorSim.getAngularPositionRotations() * _gearRatio;
    final double velocity_rps =
        Units.radiansToRotations(_motorSim.getAngularVelocityRadPerSec()) * _gearRatio;

    _talonFXSim.setRawRotorPosition(position_rot);
    _talonFXSim.setRotorVelocity(velocity_rps);

    _talonFXSim.setSupplyVoltage(12 - _talonFXSim.getSupplyCurrent() * kMotorResistance);

    if (_canCoderInvert) {
      _canCoderSim.setRawPosition(-position_rot / _gearRatio);
      _canCoderSim.setVelocity(-velocity_rps / _gearRatio);
    } else {
      _canCoderSim.setRawPosition(position_rot / _gearRatio);
      _canCoderSim.setVelocity(velocity_rps / _gearRatio);
    }
  }
}
