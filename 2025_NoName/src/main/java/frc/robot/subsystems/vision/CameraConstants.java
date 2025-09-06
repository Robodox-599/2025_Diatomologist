// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class CameraConstants {
  /* VISION */
  // FRONT LEFT CAMERA
  public static final String frontLeftCameraName = "FL_Camera";

  // CAMERA 1 POSE (X)
  public static final double frontLeftCameraPoseX = Units.inchesToMeters(7.28); //

  // CAMERA 1 POSE (Y)
  public static final double frontLeftCameraPoseY =
      Units.inchesToMeters(9.21); // (should be positive)

  // CAMERA 1 POSE (Z)
  public static final double frontLeftCameraPoseZ = Units.inchesToMeters(7.88); //

  // CAMERA 1 POSE (ROLL)
  public static final double frontLeftCameraPoseRoll = Units.degreesToRadians(0); // 0

  // CAMERA 1 POSE (PITCH)
  public static final double frontLeftCameraPosePitch = Units.degreesToRadians(-16.98); // -15

  // CAMERA 1 POSE (YAW)
  public static final double frontLeftCameraPoseYaw =
      Units.degreesToRadians(-28.66); // (should be negative)

  // FRONT RIGHT CAMERA
  public static final String frontRightCameraName = "FR_Camera";

  // CAMERA 2 POSE (X)
  public static final double frontRightCameraPoseX = Units.inchesToMeters(7.28); // 3.906

  // CAMERA 2 POSE (Y)
  public static final double frontRightCameraPoseY =
      Units.inchesToMeters(-9.21); // (should be negative)

  // CAMERA 2 POSE (Z)
  public static final double frontRightCameraPoseZ = Units.inchesToMeters(7.88);

  // CAMERA 2 POSE (ROLL)
  public static final double frontRightCameraPoseRoll = Units.degreesToRadians(0);

  // CAMERA 2 POSE (PITCH)
  public static final double frontRightCameraPosePitch = Units.degreesToRadians(-16.98); // -15

  // CAMERA 2 POSE (YAW)
  public static final double frontRightCameraPoseYaw =
      Units.degreesToRadians(28.66); // (should be positive)

  // BACK CAMERA
  public static final String backCameraName = "B_Camera";

  // CAMERA 3 POSE (X)
  public static final double backCameraPoseX = Units.inchesToMeters(0);

  // CAMERA 3 POSE (Z)
  public static final double backCameraPoseZ = Units.inchesToMeters(40.93245);

  // CAMERA 3 POSE (Y)
  public static final double backCameraPoseY = Units.inchesToMeters(0.63967);

  // CAMERA 3 POSE (ROLL)
  public static final double backCameraPoseRoll = Units.degreesToRadians(0);

  // CAMERA 3 POSE (PITCH)
  public static final double backCameraPosePitch = Units.degreesToRadians(-33.03);

  // CAMERA 3 POSE (YAW)
  public static final double backCameraPoseYaw = Units.degreesToRadians(180);
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

  public static final VisionConstants frontLeftCameraConstants =
      new VisionConstants(
          frontLeftCameraName,
          new Transform3d(
              new Translation3d(frontLeftCameraPoseX, frontLeftCameraPoseY, frontLeftCameraPoseZ),
              new Rotation3d(
                  frontLeftCameraPoseRoll, frontLeftCameraPosePitch, frontLeftCameraPoseYaw)),
          0.9);

  public static final VisionConstants frontRightCameraConstants =
      new VisionConstants(
          frontRightCameraName,
          new Transform3d(
              new Translation3d(
                  frontRightCameraPoseX, frontRightCameraPoseY, frontRightCameraPoseZ),
              new Rotation3d(
                  frontRightCameraPoseRoll, frontRightCameraPosePitch, frontRightCameraPoseYaw)),
          0.9);
  public static final VisionConstants backCameraConstants =
      new VisionConstants(
          backCameraName,
          new Transform3d(
              new Translation3d(backCameraPoseX, backCameraPoseY, backCameraPoseZ),
              new Rotation3d(backCameraPoseRoll, backCameraPosePitch, backCameraPoseYaw)),
          1.0);
  //   public static final VisionConstants cam4Constants =
  //       new VisionConstants(
  //           camera4Name,
  //           new Transform3d(
  //               new Translation3d(camera4PoseX, camera4PoseY, camera4PoseZ),
  //               new Rotation3d(camera4PoseRoll, camera4PosePitch, camera4PoseYaw)),
  //           1.0);
}
