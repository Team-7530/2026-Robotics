package frc.robot.sim;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Class to keep all the mechanism-specific objects together and out of the main example */
public class Mechanisms {
  // Visual area scale for mechanism2d
  double HEIGHT = 1; // Controls the height of the mech2d SmartDashboard
  double WIDTH = 1; // Controls the width of the mech2d SmartDashboard

  Mechanism2d mech = new Mechanism2d(WIDTH, HEIGHT);

  // ----- Rake arm (single pivot on the left side, near middle of robot)
  // Pivot placed at x=0.0 (left bumper area), y=0.5 (center/middle along Y)
  // Arm length set a bit longer than half the base so the rake extends past the bumper.
  MechanismLigament2d leftArmRoot =
      mech.getRoot("leftPivot", 0.0, 0.5)
          .append(new MechanismLigament2d("rakeArm", 0.55, 0, 8, new Color8Bit(Color.kBlue)));

  MechanismLigament2d leftRakeBase =
      leftArmRoot.append(new MechanismLigament2d("rakeBase", 0.04, 0, 6, new Color8Bit(Color.kOrange)));
  // two 90-degree over-the-bumper segments (visuals)
  MechanismLigament2d leftRakeSeg1 =
      leftRakeBase.append(new MechanismLigament2d("rakeSeg1", 0.22, -90, 6, new Color8Bit(Color.kOrange)));
  MechanismLigament2d leftRakeSeg2 =
      leftRakeSeg1.append(new MechanismLigament2d("rakeSeg2", 0.18, -90, 6, new Color8Bit(Color.kOrange)));

  // (No middle arm - design uses a single rake pivoted on the left side)

  // ----- Front turret mechanism (centered at front, ~12 inches up visually)
  MechanismLigament2d turretBase =
      mech.getRoot("turretBase", 0.5, 0.12)
          .append(new MechanismLigament2d("turretBase", 0.06, 0, 6, new Color8Bit(Color.kYellow)));
  MechanismLigament2d turretShaft =
      turretBase.append(new MechanismLigament2d("turretShaft", 0.12, 0, 6, new Color8Bit(Color.kYellow)));

    /**
     * Update mechanism visuals. Angles are provided in degrees.
     * leftArmDeg, leftRake1Deg, leftRake2Deg: rake arm joint angles
     * turretDeg: turret rotation in degrees
     */
    public void update(
            double leftArmDeg, double turretDeg) {
        // leftArmRoot.setAngle(leftArmDeg);
        leftRakeBase.setAngle(leftArmDeg);
        // leftRakeSeg1.setAngle(leftRake2Deg);
        // leftRakeSeg2.setAngle(leftRake2Deg);

        turretBase.setAngle(turretDeg);
        turretShaft.setAngle(turretDeg);

        SmartDashboard.putData("mech2d", mech); // Creates mech2d in SmartDashboard
    }
}
