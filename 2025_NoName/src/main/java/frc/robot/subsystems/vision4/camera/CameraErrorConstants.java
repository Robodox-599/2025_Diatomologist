package frc.robot.subsystems.vision4.camera;

import frc.robot.subsystems.drive.constants.TunerConstants;
import frc.robot.util.LerpTable;

public class CameraErrorConstants {
  public static final double maxZError = 0.2;
  public static final double maxAngleError =
      0.08726646259971647; // 5 degrees in radians, used for filtering out bad observations
  public static final double maxAmbiguity = 0.25;

  public static final LerpTable LINEAR_VELOCITY_STD_DEV_COEFFICIENT =
      new LerpTable(
          new LerpTable.LerpTableEntry(0.0, 1.0),
          new LerpTable.LerpTableEntry(2.5, 1.25),
          new LerpTable.LerpTableEntry(TunerConstants.MAX_LINEAR_SPEED, 10));

  public static final LerpTable ANGULAR_VELOCITY_STD_DEV_COEFFICIENT =
      new LerpTable(
          new LerpTable.LerpTableEntry(0.0, 1.0),
          new LerpTable.LerpTableEntry(7.0, 1.54),
          new LerpTable.LerpTableEntry(12.0, 100.0));
  public static final LerpTable DISTANCE_STD_DEV_COEFFICIENT =
      new LerpTable(
          new LerpTable.LerpTableEntry(0.0, 1.0),
          new LerpTable.LerpTableEntry(0.65, 1.0),
          new LerpTable.LerpTableEntry(1.5, 1.43),
          new LerpTable.LerpTableEntry(2.5, 2.5),
          new LerpTable.LerpTableEntry(5.0, 4.0),
          new LerpTable.LerpTableEntry(8.0, 20.0));
}
