package frc.robot.commands;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.Supplier;

public class AutoAlignToField {
  public static Pose2d getNearestBranchPosition(
      Supplier<Pose2d> robotPoseSupplier, boolean useLeftBranch) {
    Pose2d robotPose = robotPoseSupplier.get();
    Pose2d nearestFace = null;
    double minDistance = Double.MAX_VALUE;

    // Find the nearest center face and its index
    for (int i = 0; i < 6; i++) {
      Pose2d centerFace = AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[i]);

      double distance = robotPose.getTranslation().getDistance(centerFace.getTranslation());

      if (distance < minDistance) {
        minDistance = distance;
        nearestFace = centerFace;
      }
    }

    double adjustX = Units.inchesToMeters(16.75 + 1); // inches from reef face (bot radius + 1 inch)
    double adjustY = Units.inchesToMeters(6.469); // inches from center (exact)

    // Apply the transformation based on left/right boolean
    Pose2d branchPosition =
        new Pose2d(nearestFace.getTranslation(), nearestFace.getRotation())
            .transformBy(
                new Transform2d(adjustX, useLeftBranch ? -adjustY : adjustY, new Rotation2d()));

    // The result is now in the same position as the corresponding branchPositions entry
    Pose2d targetPose = branchPosition;
    DogLog.log("ClosestFace/TargetPose", targetPose);
    DogLog.log("ClosestFace/RobotPose", robotPose);
    return targetPose;
  }

  public static Pose2d getNearestReefFacePosition(Supplier<Pose2d> robotPoseSupplier) {
    Pose2d robotPose = robotPoseSupplier.get();
    Pose2d nearestFace = null;
    double minDistance = Double.MAX_VALUE;

    // Find the nearest center face and its index
    for (int i = 0; i < 6; i++) {
      Pose2d centerFace = AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[i]);

      double distance = robotPose.getTranslation().getDistance(centerFace.getTranslation());

      if (distance < minDistance) {
        minDistance = distance;
        nearestFace = centerFace;
      }
    }

    double adjustX = Units.inchesToMeters(16.75 + 1); // inches from reef face (bot radius + 1 inch)

    Pose2d nearestReefFacePosition =
        new Pose2d(nearestFace.getTranslation(), nearestFace.getRotation())
            .transformBy(new Transform2d(adjustX, 0, new Rotation2d()));

    // The result is now in the same position as the corresponding branchPositions entry
    Pose2d targetPose = nearestReefFacePosition;
    DogLog.log("ClosestFace/TargetPose", targetPose);
    DogLog.log("ClosestFace/RobotPose", robotPose);
    return targetPose;
  }

  public static Command alignToNearestLeftBranch(CommandSwerveDrivetrain drive) {
    return drive.moveToPoint(
        () ->
            getNearestBranchPosition(() -> drive.getState().Pose, true)
                .plus(new Transform2d(new Translation2d(), new Rotation2d(Math.PI))));
  }

  public static Command alignToNearestRightBranch(CommandSwerveDrivetrain drive) {
    return drive.moveToPoint(
        () ->
            getNearestBranchPosition(() -> drive.getState().Pose, false)
                .plus(new Transform2d(new Translation2d(), new Rotation2d(Math.PI))));
  }

  public static Command alignToNearestReefFace(CommandSwerveDrivetrain drive) {
    return drive.moveToPoint(
        () ->
            getNearestReefFacePosition(() -> drive.getState().Pose)
                .plus(new Transform2d(new Translation2d(), new Rotation2d(Math.PI))));
  }
}
