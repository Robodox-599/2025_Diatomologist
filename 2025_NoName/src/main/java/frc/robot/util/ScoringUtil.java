package frc.robot.util;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants;
import java.util.function.Supplier;

public class ScoringUtil {
  public static Pose2d getNearestBranchPosition(
      Supplier<Pose2d> robotPoseSupplier, boolean useLeftBranch, Translation2d coralOffset) {
    Pose2d robotPose = robotPoseSupplier.get();
    Pose2d nearestFace = null;
    double minDistance = Double.MAX_VALUE;

    // Find the nearest center face and its index
    for (int i = 0; i < FieldConstants.Reef.centerFaces.length; i++) {
      Pose2d centerFace = AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[i]);
      double distance = robotPose.getTranslation().getDistance(centerFace.getTranslation());

      if (distance < minDistance) {
        minDistance = distance;
        nearestFace = centerFace;
      }
    }
    if (nearestFace != null) {
      // Calculate the branch position using the same offsets as in FieldConstants
      Pose2d poseDirection = new Pose2d(FieldConstants.Reef.center, nearestFace.getRotation());

      double adjustX = Units.inchesToMeters(30.738);
      double adjustY = Units.inchesToMeters(6.469);

      // Apply the transformation based on left/right boolean
      Pose2d branchPosition =
          new Pose2d(poseDirection.getTranslation(), poseDirection.getRotation())
              .transformBy(
                  new Transform2d(
                      adjustX, useLeftBranch ? -adjustY : adjustY, nearestFace.getRotation()));

      // The result is now in the same position as the corresponding branchPositions entry
      Pose2d targetPose = branchPosition;
      DogLog.log("ClosestFace/TargetPose", targetPose);
      DogLog.log("ClosestFace/RobotPose", robotPose);
      return targetPose;
    } else {
      // Fallback to the reef center if no face was found
      Pose2d targetPose = new Pose2d(FieldConstants.Reef.center, robotPose.getRotation());
      DogLog.log("ClosestFace/TargetPose", targetPose);
      DogLog.log("ClosestFace/RobotPose", robotPose);
      return targetPose;
    }
  }
}
