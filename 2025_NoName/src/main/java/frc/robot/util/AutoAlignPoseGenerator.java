package frc.robot.util;

import static frc.robot.FieldConstants.*;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AutoAlignPoseGenerator {
  private static int algaeReefFaceIndex = 0;

  /**
   * Finds nearest branch position
   *
   * @param robotPose robot pose
   * @param useLeftBranch true if left branch, false if right branch
   */
  public static Pose2d getNearestBranchPosition(Pose2d robotPose, boolean useLeftBranch) {
    double minDistance = Double.MAX_VALUE;
    int nearestFaceIndex = -1;

    // Find the nearest center face and its index
    for (int i = 0; i < 6; i++) {
      Pose2d centerFace =
          DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
              ? REEF_BLUE_MIDDLE[i]
              : REEF_RED_MIDDLE[i];

      double distance = robotPose.getTranslation().getDistance(centerFace.getTranslation());

      if (distance < minDistance) {
        minDistance = distance;
        nearestFaceIndex = i;
      }
    }

    Pose2d targetPose;
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      targetPose =
          useLeftBranch ? REEF_BLUE_LEFT[nearestFaceIndex] : REEF_BLUE_RIGHT[nearestFaceIndex];
    } else {
      targetPose =
          useLeftBranch ? REEF_RED_LEFT[nearestFaceIndex] : REEF_RED_RIGHT[nearestFaceIndex];
    }

    DogLog.log("ClosestFace/TargetPose", targetPose);
    DogLog.log("ClosestFace/RobotPose", robotPose);
    return targetPose;
  }

  /**
   * Finds nearest branch position
   *
   * @param robotPose robot pose
   * @param useLeftBranch true if left branch, false if right branch
   */
  public static Pose2d getNearestTroughPosition(Pose2d robotPose, int troughIndex) {
    double minDistance = Double.MAX_VALUE;
    int nearestFaceIndex = -1;

    // Find the nearest center face and its index
    for (int i = 0; i < 6; i++) {
      Pose2d centerFace =
          DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
              ? REEF_BLUE_MIDDLE[i]
              : REEF_RED_MIDDLE[i];

      double distance = robotPose.getTranslation().getDistance(centerFace.getTranslation());

      if (distance < minDistance) {
        minDistance = distance;
        nearestFaceIndex = i;
      }
    }

    Pose2d targetPose = new Pose2d();
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      if (troughIndex == 1) { // left
        targetPose =
            REEF_BLUE_MIDDLE[nearestFaceIndex].transformBy(
                new Transform2d(0.0, 0.45, new Rotation2d(0)));
      } else if (troughIndex == 2) { // middle
        targetPose = REEF_BLUE_MIDDLE[nearestFaceIndex];
      } else if (troughIndex == 3) { // right
        targetPose =
            REEF_BLUE_MIDDLE[nearestFaceIndex].transformBy(
                new Transform2d(0.0, -0.45, new Rotation2d(0)));
      }
    } else {
      if (troughIndex == 1) { // left
        targetPose =
            REEF_RED_MIDDLE[nearestFaceIndex].transformBy(
                new Transform2d(0.0, 0.45, new Rotation2d(0)));
      } else if (troughIndex == 2) { // middle
        targetPose = REEF_RED_MIDDLE[nearestFaceIndex];
      } else if (troughIndex == 3) { // right
        targetPose =
            REEF_RED_MIDDLE[nearestFaceIndex].transformBy(
                new Transform2d(0.0, -0.45, new Rotation2d(0)));
      }
    }

    targetPose = targetPose.transformBy(new Transform2d(0.38, 0, new Rotation2d(0)));

    DogLog.log("ClosestFace/TargetPose", targetPose);
    DogLog.log("ClosestFace/RobotPose", robotPose);
    return targetPose;
  }

  /**
   * Finds nearest reef face position for algae
   *
   * @param robotPose robot pose
   * @param shiftBackFromReefFace true if the target pose should be shifted back from the reef face
   */
  public static Pose2d getNearestAlgaeReefFacePosition(
      Pose2d robotPose, boolean shiftBackFromReefFace) {
    Pose2d nearestFace = null;
    double minDistance = Double.MAX_VALUE;

    // Find the nearest center face and its index
    for (int i = 0; i < 6; i++) {
      Pose2d centerFace =
          DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
              ? REEF_BLUE_MIDDLE[i]
              : REEF_RED_MIDDLE[i];

      double distance = robotPose.getTranslation().getDistance(centerFace.getTranslation());

      if (distance < minDistance) {
        minDistance = distance;
        nearestFace = centerFace;
        algaeReefFaceIndex = i;
      }
    }

    if (shiftBackFromReefFace) {
      nearestFace = nearestFace.transformBy(new Transform2d(0.4, 0, new Rotation2d(0)));
    }

    DogLog.log("ClosestFace/TargetPose", nearestFace);
    DogLog.log("ClosestFace/RobotPose", robotPose);
    return nearestFace;
  }

  public static int getReefFaceIndex() {
    return algaeReefFaceIndex;
  }
}
