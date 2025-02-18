package frc.robot.subsystems.commands;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.endefector.endefectorrollers.Rollers;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.Supplier;

public class AutoAlignToField {
  public static Pose2d getNearestBranchPosition(
      Supplier<Pose2d> robotPoseSupplier, boolean useLeftBranch, Translation2d coralOffset) {
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

    double adjustX = Units.inchesToMeters(30.738);
    double adjustY = Units.inchesToMeters(6.469);

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

  public static Command alignToNearestLeftReef(Drive drive, Rollers rollers) {
    var driveToPose =
        new DriveToPose(
            drive,
            () ->
                getNearestBranchPosition(
                    () -> drive.getPose(),
                    true,
                    new Translation2d(rollers.getCoralDistance(), new Rotation2d())));

    return driveToPose.until(() -> (driveToPose.withinTolerance() || driveToPose.atGoal()));
  }

  public static Command alignToNearestRightReef(Drive drive, Rollers rollers) {
    var driveToPose =
        new DriveToPose(
            drive,
            () ->
                getNearestBranchPosition(
                    () -> drive.getPose(),
                    false,
                    new Translation2d(rollers.getCoralDistance(), new Rotation2d())));

    return driveToPose.until(() -> (driveToPose.withinTolerance() || driveToPose.atGoal()));
  }
}
