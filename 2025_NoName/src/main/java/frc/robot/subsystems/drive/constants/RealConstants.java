package frc.robot.subsystems.drive.constants;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.Module.ModuleConstants;
import frc.robot.subsystems.vision.VisionConstants;

// import frc.robot.subsystems.vision.VisionConstants;

public class RealConstants {

  public static final double Module0AbsoluteEncoderOffset = -0.2685546875; // FL
  public static final double Module1AbsoluteEncoderOffset = -0.1533203125; // FR
  public static final double Module2AbsoluteEncoderOffset = -0.02783203125; // BL
  public static final double Module3AbsoluteEncoderOffset = 0.140625; // BR

  public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);

  public static final double ODOMETRY_FREQUENCY = 250.0;

  public static final double DRIVE_GEAR_RATIO = 5.36;

  // TURNING GEAR RATIO
  public static final double TURN_GEAR_RATIO = (150.0 / 7.0);

  public static final double MAX_LINEAR_SPEED = 4.69;
  public static final double TRACK_WIDTH_X = Units.inchesToMeters(20.75);
  public static final double TRACK_WIDTH_Y = Units.inchesToMeters(20.75);
  public static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  public static final double MAX_ANGULAR_SPEED = (MAX_LINEAR_SPEED * 0.5) / DRIVE_BASE_RADIUS;
  public static final double MAX_LINEAR_ACCELERATION = 8.0;
  public static final double MAX_ANGULAR_ACCELERATION = MAX_LINEAR_ACCELERATION / DRIVE_BASE_RADIUS;
  public static final double MAX_AUTOAIM_SPEED = MAX_LINEAR_SPEED / 4;

  public static final boolean IS_TURN_MOTOR_INVERTED = true;
  public static final double TURN_STATOR_CURRENT_LIMIT = 40.0;
  public static final double DRIVE_ROTOR_TO_METERS =
      (RealConstants.DRIVE_GEAR_RATIO) / (WHEEL_RADIUS * 2 * Math.PI);

  // Both sets of gains need to be tuned to our robot. make sure we tune this with torque control
  // foc for both modules.
  private static final Slot0Configs steerGains =
      new Slot0Configs()
          .withKP(105)
          .withKI(0)
          .withKD(2.15)
          .withKS(0.14)
          .withKV(0.4)
          .withKA(0)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

  private static final Slot0Configs driveGains =
      new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKS(0).withKV(0.124);

  // record classes passed in to create modules.
  public static final ModuleConstants frontLeft =
      new ModuleConstants(
          "Front Left",
          0,
          1,
          2,
          "DongleDriveCANivore",
          Rotation2d.fromRotations(Module0AbsoluteEncoderOffset),
          steerGains,
          driveGains,
          WHEEL_RADIUS,
          false,
          false,
          false);
  public static final ModuleConstants frontRight =
      new ModuleConstants(
          "Front Right",
          3,
          4,
          5,
          "DongleDriveCANivore",
          Rotation2d.fromRotations(Module1AbsoluteEncoderOffset),
          steerGains,
          driveGains,
          WHEEL_RADIUS,
          false,
          false,
          false);
  public static final ModuleConstants backLeft =
      new ModuleConstants(
          "Back Left",
          6,
          7,
          8,
          "DongleDriveCANivore",
          Rotation2d.fromRotations(Module2AbsoluteEncoderOffset),
          steerGains,
          driveGains,
          WHEEL_RADIUS,
          false,
          false,
          false);
  public static final ModuleConstants backRight =
      new ModuleConstants(
          "Back Right",
          9,
          10,
          11,
          "DongleDriveCANivore",
          Rotation2d.fromRotations(Module3AbsoluteEncoderOffset),
          steerGains,
          driveGains,
          WHEEL_RADIUS,
          false,
          false,
          false);

  public static final String camera1Name = "FL_Camera";

  // CAMERA 1 POSE (X)
  public static final double camera1PoseX = Units.inchesToMeters(6.79209841);

  // CAMERA 1 POSE (Z)
  public static final double camera1PoseZ = Units.inchesToMeters(4.99763044);

  // CAMERA 1 POSE (Y)
  public static final double camera1PoseY = Units.inchesToMeters(9.15983669);

  // CAMERA 1 POSE (ROLL)
  public static final double camera1PoseRoll = Units.degreesToRadians(0);

  // CAMERA 1 POSE (PITCH)
  public static final double camera1PosePitch = Units.degreesToRadians(-15);

  // CAMERA 1 POSE (YAW)
  public static final double camera1PoseYaw = Units.degreesToRadians(-28.6588);

  public static final String camera2Name = "FR_Camera";

  // CAMERA 2 POSE (X)
  public static final double camera2PoseX = Units.inchesToMeters(6.79209841);

  // CAMERA 2 POSE (Z)
  public static final double camera2PoseZ = Units.inchesToMeters(4.99763044);

  // CAMERA 2 POSE (Y)
  public static final double camera2PoseY = Units.inchesToMeters(-9.15983669);

  // CAMERA 2 POSE (ROLL)
  public static final double camera2PoseRoll = Units.degreesToRadians(0);

  // CAMERA 2 POSE (PITCH)
  public static final double camera2PosePitch = Units.degreesToRadians(-15);

  // CAMERA 2 POSE (YAW)
  public static final double camera2PoseYaw = Units.degreesToRadians(28.6588);

  public static final String camera3Name = "BL_Camera";

  // CAMERA 3 POSE (X)
  public static final double camera3PoseX = Units.inchesToMeters(12.37664406);

  // CAMERA 3 POSE (Z)
  public static final double camera3PoseZ = Units.inchesToMeters(4.88755783);

  // CAMERA 3 POSE (Y)
  public static final double camera3PoseY = Units.inchesToMeters(7.86625496);

  // CAMERA 3 POSE (ROLL)
  public static final double camera3PoseRoll = Units.degreesToRadians(0);

  // CAMERA 3 POSE (PITCH)
  public static final double camera3PosePitch = Units.degreesToRadians(-15);

  // CAMERA 3 POSE (YAW)
  public static final double camera3PoseYaw = Units.degreesToRadians(28.6588);

  public static final String camera4Name = "BR_Camera";

  // CAMERA 3 POSE (X)
  public static final double camera4PoseX = Units.inchesToMeters(12.37664406);

  // CAMERA 3 POSE (Z)
  public static final double camera4PoseZ = Units.inchesToMeters(4.88755783);

  // CAMERA 3 POSE (Y)
  public static final double camera4PoseY = Units.inchesToMeters(-7.86625496);

  // CAMERA 3 POSE (ROLL)
  public static final double camera4PoseRoll = Units.degreesToRadians(0);

  // CAMERA 3 POSE (PITCH)
  public static final double camera4PosePitch = Units.degreesToRadians(-15);

  // CAMERA 3 POSE (YAW)
  public static final double camera4PoseYaw = Units.degreesToRadians(-28.6588);

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
  public static final VisionConstants cam4Constants =
      new VisionConstants(
          camera4Name,
          new Transform3d(
              new Translation3d(camera4PoseX, camera4PoseY, camera4PoseZ),
              new Rotation3d(camera4PoseRoll, camera4PosePitch, camera4PoseYaw)),
          1.0);
}
