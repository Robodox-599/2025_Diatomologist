// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.VisionConstants;

public class CameraConstants {
  /* VISION */
  public static final String camera1Name = "FL_Camera";

  // CAMERA 1 POSE (X)
  public static final double camera1PoseX = Units.inchesToMeters(9.2096);

  // CAMERA 1 POSE (Z)
  public static final double camera1PoseZ = Units.inchesToMeters(7.8473);

  // CAMERA 1 POSE (Y)
  public static final double camera1PoseY = Units.inchesToMeters(6.7010);

  // CAMERA 1 POSE (ROLL)
  public static final double camera1PoseRoll = Units.degreesToRadians(0);

  // CAMERA 1 POSE (PITCH)
  public static final double camera1PosePitch = Units.degreesToRadians(-15);

  // CAMERA 1 POSE (YAW)
  public static final double camera1PoseYaw = Units.degreesToRadians(-32.049);

  public static final String camera2Name = "FR_Camera";

  // CAMERA 2 POSE (X)
  public static final double camera2PoseX = Units.inchesToMeters(9.2379);

  // CAMERA 2 POSE (Z)
  public static final double camera2PoseZ = Units.inchesToMeters(7.8473);

  // CAMERA 2 POSE (Y)
  public static final double camera2PoseY = Units.inchesToMeters(-7.0619);

  // CAMERA 2 POSE (ROLL)
  public static final double camera2PoseRoll = Units.degreesToRadians(0);

  // CAMERA 2 POSE (PITCH)
  public static final double camera2PosePitch = Units.degreesToRadians(-15);

  // CAMERA 2 POSE (YAW)
  public static final double camera2PoseYaw = Units.degreesToRadians(32.007);

  public static final String camera3Name = "B_Camera";

  // CAMERA 3 POSE (X)
  public static final double camera3PoseX = Units.inchesToMeters(0);

  // CAMERA 3 POSE (Z)
  public static final double camera3PoseZ = Units.inchesToMeters(40.93245);

  // CAMERA 3 POSE (Y)
  public static final double camera3PoseY = Units.inchesToMeters(0.63967);

  // CAMERA 3 POSE (ROLL)
  public static final double camera3PoseRoll = Units.degreesToRadians(0);

  // CAMERA 3 POSE (PITCH)
  public static final double camera3PosePitch = Units.degreesToRadians(-33.03);

  // CAMERA 3 POSE (YAW)
  public static final double camera3PoseYaw = Units.degreesToRadians(180);
  //   public static final String camera4Name = "BR_Camera";

  //   // CAMERA 4 POSE (X)
  //   public static final double camera4PoseX = Units.inchesToMeters(12.37664406);

  //   // CAMERA 4 POSE (Z)
  //   public static final double camera4PoseZ = Units.inchesToMeters(4.88755783);

  //   // CAMERA 4 POSE (Y)
  //   public static final double camera4PoseY = Units.inchesToMeters(-7.86625496);

  //   // CAMERA 4 POSE (ROLL)
  //   public static final double camera4PoseRoll = Units.degreesToRadians(0);

  //   // CAMERA 4 POSE (PITCH)
  //   public static final double camera4PosePitch = Units.degreesToRadians(-15);

  //   // CAMERA 4 POSE (YAW)
  //   public static final double camera4PoseYaw = Units.degreesToRadians(-28.6588);

  public static final VisionConstants cam1Constants =
      new VisionConstants(
          camera1Name,
          new Transform3d(
              new Translation3d(camera1PoseX, camera1PoseY, camera1PoseZ),
              new Rotation3d(camera1PoseRoll, camera1PosePitch, camera1PoseYaw)),
          1.0);

  public static final VisionConstants cam2Constants =
      new VisionConstants(
          camera2Name,
          new Transform3d(
              new Translation3d(camera2PoseX, camera2PoseY, camera2PoseZ),
              new Rotation3d(camera2PoseRoll, camera2PosePitch, camera2PoseYaw)),
          1.0);
  public static final VisionConstants cam3Constants =
      new VisionConstants(
          camera3Name,
          new Transform3d(
              new Translation3d(camera3PoseX, camera3PoseY, camera3PoseZ),
              new Rotation3d(camera3PoseRoll, camera3PosePitch, camera3PoseYaw)),
          1.0);
  //   public static final VisionConstants cam4Constants =
  //       new VisionConstants(
  //           camera4Name,
  //           new Transform3d(
  //               new Translation3d(camera4PoseX, camera4PoseY, camera4PoseZ),
  //               new Rotation3d(camera4PoseRoll, camera4PosePitch, camera4PoseYaw)),
  //           1.0);
}
