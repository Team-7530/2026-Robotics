package frc.robot.sim;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Class to keep all the mechanism-specific objects together and out of the main example */
public class Mechanisms {
  double HEIGHT = 1; // Controls the height of the mech2d SmartDashboard
  double WIDTH = 1; // Controls the height of the mech2d SmartDashboard

  Mechanism2d mech = new Mechanism2d(WIDTH, HEIGHT);
  /* arm rotor rotor Ligaments */
  MechanismLigament2d rotorArm =
      mech.getRoot("rotorArmPivotPoint", 0.5, 0.2)
          .append(new MechanismLigament2d("rotorArm", .04, 0, 0, new Color8Bit(Color.kAliceBlue)));

  MechanismLigament2d rotorArmSide1 =
      rotorArm.append(
          new MechanismLigament2d(
              "rotorArmSide1", 0.038267, 120, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d rotorArmSide2 =
      rotorArmSide1.append(
          new MechanismLigament2d(
              "rotorArmSide2", 0.038267, 60, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d rotorArmSide3 =
      rotorArmSide2.append(
          new MechanismLigament2d(
              "rotorArmSide3", 0.038267, 60, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d rotorArmSide4 =
      rotorArmSide3.append(
          new MechanismLigament2d(
              "rotorArmSide4", 0.038267, 60, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d rotorArmSide5 =
      rotorArmSide4.append(
          new MechanismLigament2d(
              "rotorArmSide5", 0.038267, 60, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d rotorArmSide6 =
      rotorArmSide5.append(
          new MechanismLigament2d(
              "rotorArmSide6", 0.038267, 60, 6, new Color8Bit(Color.kAliceBlue)));

  /* Arm ligaments */
  MechanismLigament2d ccArm =
      mech.getRoot("armPivotPoint", 0.5, 0.2)
          .append(new MechanismLigament2d("ccArm", .1, 0, 0, new Color8Bit(Color.kAntiqueWhite)));

  MechanismLigament2d ccSide1 =
      ccArm.append(
          new MechanismLigament2d(
              "ccSide1", 0.076535, 112.5, 6, new Color8Bit(Color.kAntiqueWhite)));
  MechanismLigament2d ccSide2 =
      ccSide1.append(
          new MechanismLigament2d("ccSide2", 0.076535, 45, 6, new Color8Bit(Color.kAntiqueWhite)));
  MechanismLigament2d ccSide3 =
      ccSide2.append(
          new MechanismLigament2d("ccSide3", 0.076535, 45, 6, new Color8Bit(Color.kAntiqueWhite)));
  MechanismLigament2d ccSide4 =
      ccSide3.append(
          new MechanismLigament2d("ccSide4", 0.076535, 45, 6, new Color8Bit(Color.kAntiqueWhite)));
  MechanismLigament2d ccSide5 =
      ccSide4.append(
          new MechanismLigament2d("ccSide5", 0.076535, 45, 6, new Color8Bit(Color.kAntiqueWhite)));
  MechanismLigament2d ccSide6 =
      ccSide5.append(
          new MechanismLigament2d("ccSide6", 0.076535, 45, 6, new Color8Bit(Color.kAntiqueWhite)));
  MechanismLigament2d ccSide7 =
      ccSide6.append(
          new MechanismLigament2d("ccSide7", 0.076535, 45, 6, new Color8Bit(Color.kAntiqueWhite)));
  MechanismLigament2d ccSide8 =
      ccSide7.append(
          new MechanismLigament2d("ccSide8", 0.076535, 45, 6, new Color8Bit(Color.kAntiqueWhite)));
  MechanismLigament2d ccArmLen =
      ccArm.append(new MechanismLigament2d("ccArmLen", 0.3, 0, 8, new Color8Bit(Color.kBlue)));

  /* Wrist Mechanism Ligaments */
  MechanismLigament2d ccWrist =
      ccArmLen.append(new MechanismLigament2d("ccWrist", .025, 0, 6, new Color8Bit(Color.kOrange)));
  MechanismLigament2d wristSide2 =
      ccWrist.append(
          new MechanismLigament2d("wristSide2", 0.2, -90, 6, new Color8Bit(Color.kOrange)));
  MechanismLigament2d wristSide3 =
      wristSide2.append(
          new MechanismLigament2d("wristSide3", 0.05, -90, 6, new Color8Bit(Color.kOrange)));
  MechanismLigament2d wristSide4 =
      wristSide3.append(
          new MechanismLigament2d("wristSide4", 0.2, -90, 6, new Color8Bit(Color.kOrange)));
  MechanismLigament2d wristSide5 =
      wristSide4.append(
          new MechanismLigament2d("wristSide5", 0.025, -90, 6, new Color8Bit(Color.kOrange)));

  /* climber Mechanism Ligaments */
  MechanismLigament2d rotorClimb =
      mech.getRoot("rotorClimberPivotPoint", 0.75, 0.3)
          .append(
              new MechanismLigament2d("rotorClimb", .04, 0, 0, new Color8Bit(Color.kAliceBlue)));

  MechanismLigament2d rotorClimbSide1 =
      rotorClimb.append(
          new MechanismLigament2d(
              "rotorClimbSide1", 0.038267, 120, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d rotorClimbSide2 =
      rotorClimbSide1.append(
          new MechanismLigament2d(
              "rotorClimbSide2", 0.038267, 60, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d rotorClimbSide3 =
      rotorClimbSide2.append(
          new MechanismLigament2d(
              "rotorClimbSide3", 0.038267, 60, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d rotorClimbSide4 =
      rotorClimbSide3.append(
          new MechanismLigament2d(
              "rotorClimbSide4", 0.038267, 60, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d rotorClimbSide5 =
      rotorClimbSide4.append(
          new MechanismLigament2d(
              "rotorClimbSide5", 0.038267, 60, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d rotorClimbSide6 =
      rotorClimbSide5.append(
          new MechanismLigament2d(
              "rotorClimbSide6", 0.038267, 60, 6, new Color8Bit(Color.kAliceBlue)));

  MechanismLigament2d ccClimb =
      mech.getRoot("climbPivotPoint", 0.75, 0.3)
          .append(new MechanismLigament2d("ccClimb", .15, 90, 6, new Color8Bit(Color.kPurple)));
  MechanismLigament2d climbSide1 =
      ccClimb.append(
          new MechanismLigament2d("climbSide1", .1, -90, 6, new Color8Bit(Color.kPurple)));
  MechanismLigament2d climbSide2 =
      ccClimb.append(
          new MechanismLigament2d("climbSide2", 0.25, 180, 6, new Color8Bit(Color.kPurple)));

  public void update(
      double armRotorPos,
      double armEncoderPos,
      double wristEncoderPos,
      double climbRotorPos,
      double climbEncoderPos) {
    // BaseStatusSignal.refreshAll(armRotorPosition, armEncoderPosition, wristEncoderPosition);
    rotorArm.setAngle(armRotorPos * 360);
    ccArm.setAngle(armEncoderPos * 360);
    ccWrist.setAngle(wristEncoderPos * 360);
    rotorClimb.setAngle(climbRotorPos * 360);
    ccClimb.setAngle(100 + climbEncoderPos * 360);
    SmartDashboard.putData("mech2d", mech); // Creates mech2d in SmartDashboard
  }
}
