package frc.robot.subsystems.drive.constants;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.Module.ModuleConstants;

// import frc.robot.subsystems.vision.VisionConstants;

public class RealConstants {
  public static final double Module0AbsoluteEncoderOffset = 0.15087890625; // FL
  public static final double Module1AbsoluteEncoderOffset = 0.363525390625; // FR
  public static final double Module2AbsoluteEncoderOffset = 0.23388671875; // BL
  public static final double Module3AbsoluteEncoderOffset = -0.467529296875; // BR

  public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);

  public static final double ODOMETRY_FREQUENCY = 250.0;

  public static final double DRIVE_GEAR_RATIO = 5.9;

  // TURNING GEAR RATIO
  public static final double TURN_GEAR_RATIO = (150.0 / 7.0);
  public static final double MAX_LINEAR_SPEED = Units.feetToMeters(16.5);
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
  public static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS; // meters
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
          "BarryDriveCANivore",
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
          "BarryDriveCANivore",
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
          "BarryDriveCANivore",
          Rotation2d.fromRotations(Module2AbsoluteEncoderOffset),
          steerGains,
          driveGains,
          WHEEL_RADIUS,
          false,
          false,
          true);
  public static final ModuleConstants backRight =
      new ModuleConstants(
          "Back Right",
          9,
          10,
          11,
          "BarryDriveCANivore",
          Rotation2d.fromRotations(Module3AbsoluteEncoderOffset),
          steerGains,
          driveGains,
          WHEEL_RADIUS,
          false,
          false,
          false);

  //   public static final String cameraName = "FR Camera";

  //   // CAMERA 3 POSE (X)
  //   public static final double cameraPoseX = Units.inchesToMeters(0);

  //   // CAMERA 3 POSE (Y)
  //   public static final double cameraPoseY = Units.inchesToMeters(1);

  //   // CAMERA 3 POSE (Z)
  //   public static final double cameraPoseZ = Units.inchesToMeters(21);

  //   // CAMERA 3 POSE (ROLL)
  //   public static final double cameraPoseRoll = Units.degreesToRadians(0);

  //   // CAMERA 3 POSE (PITCH)
  //   public static final double cameraPosePitch = Units.degreesToRadians(-28);

  //   // CAMERA 3 POSE (YAW)
  //   public static final double cameraPoseYaw = Units.degreesToRadians(0);

  //   public static final VisionConstants camConstants =
  //       new VisionConstants(
  //           cameraName,
  //           new Transform3d(
  //               new Translation3d(cameraPoseX, cameraPoseY, cameraPoseZ),
  //               new Rotation3d(cameraPoseRoll, cameraPosePitch, cameraPoseYaw)),
  //           1.0);
}
