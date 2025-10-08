package frc.robot.util;

import static frc.robot.FieldConstants.*;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AutoAlignPoseGenerator {
  public static final double L4_REEF_FACE_OFFSET =
      0.0; // distance from reef face to scoring for L4 in meters
  public static final double L4_REEF_FACE_OFFSET_AUTO =
      -0.09; // distance from reef face to scoring for L4 in meters
  public static final double L2_L3_REEF_FACE_OFFSET =
      -0.27; // distance from reef face to scoring for L2 and L3 in meters
  public static final double L1_REEF_FACE_OFFSET =
      -0.3; // distance from reef face to scoring for L1 in meters
  public static final double ALGAE_REEF_FACE_OFFSET =
      0.0; // distance from reef face to scoring for algae in meters
  public static int nearestReefFaceIndex = 0;

  public static void updateNearestReefFaceIndex(Pose2d robotPose) {
    nearestReefFaceIndex = calculateNearestReefFaceIndex(robotPose);
    DogLog.log("AutoAlignPoseGenerator/NearestFaceIndex", nearestReefFaceIndex);
  }

  public static int getNearestReefFaceIndex() {
    return nearestReefFaceIndex;
  }

  private static int calculateNearestReefFaceIndex(Pose2d robotPose) {
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
    return nearestFaceIndex;
  }

  public static Pose2d getNearestL2orL3BranchPosition(Pose2d robotPose, boolean useLeftBranch) {
    Pose2d branchPose = getNearestBranchPosition(robotPose, useLeftBranch);
    Pose2d targetPose =
        branchPose.transformBy(new Transform2d(L2_L3_REEF_FACE_OFFSET, 0, new Rotation2d(0)));
    return targetPose;
  }

  public static Pose2d getNearestL4BranchPosition(Pose2d robotPose, boolean useLeftBranch) {
    Pose2d branchPose = getNearestBranchPosition(robotPose, useLeftBranch);
    Pose2d targetPose =
        branchPose.transformBy(new Transform2d(L4_REEF_FACE_OFFSET, 0, new Rotation2d(0)));
    return targetPose;
  }

  public static Pose2d getNearestL4BranchPositionAuto(Pose2d robotPose, boolean useLeftBranch) {
    Pose2d branchPose = getNearestBranchPosition(robotPose, useLeftBranch);
    Pose2d targetPose =
        branchPose.transformBy(new Transform2d(L4_REEF_FACE_OFFSET_AUTO, 0, new Rotation2d(0)));
    return targetPose;
  }

  /**
   * Finds nearest branch position
   *
   * @param robotPose robot pose
   * @param useLeftBranch true if left branch, false if right branch
   */
  private static Pose2d getNearestBranchPosition(Pose2d robotPose, boolean useLeftBranch) {
    Pose2d targetPose;
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      targetPose =
          useLeftBranch
              ? REEF_BLUE_LEFT[nearestReefFaceIndex]
              : REEF_BLUE_RIGHT[nearestReefFaceIndex];
    } else {
      targetPose =
          useLeftBranch
              ? REEF_RED_LEFT[nearestReefFaceIndex]
              : REEF_RED_RIGHT[nearestReefFaceIndex];
    }
    DogLog.log("AutoAlignPoseGenerator/TargetPose", targetPose);
    DogLog.log("AutoAlignPoseGenerator/RobotPose", robotPose);
    DogLog.log(
        "AutoAlignPoseGenerator/Alliance",
        DriverStation.getAlliance().orElse(Alliance.Blue).toString());
    return targetPose;
  }

  /**
   * Finds nearest branch position
   *
   * @param robotPose robot pose
   * @param useLeftBranch true if left branch, false if right branch
   */
  public static Pose2d getNearestTroughPosition(Pose2d robotPose, int troughIndex) {

    Pose2d targetPose = new Pose2d();
    if (troughIndex == 1) { // left
      targetPose =
          DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
              ? REEF_BLUE_MIDDLE[nearestReefFaceIndex].transformBy(
                  new Transform2d(0.0, 0.3, new Rotation2d(0)))
              : REEF_RED_MIDDLE[nearestReefFaceIndex].transformBy(
                  new Transform2d(0.0, 0.3, new Rotation2d(0)));
    } else if (troughIndex == 2) { // middle left
      targetPose =
          DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
              ? REEF_BLUE_MIDDLE[nearestReefFaceIndex].transformBy(
                  new Transform2d(0.0, 0.06, new Rotation2d(0)))
              : REEF_RED_MIDDLE[nearestReefFaceIndex].transformBy(
                  new Transform2d(0.0, 0.06, new Rotation2d(0)));
    } else if (troughIndex == 3) { // middle right
      targetPose =
          DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
              ? REEF_BLUE_MIDDLE[nearestReefFaceIndex].transformBy(
                  new Transform2d(0.0, -0.06, new Rotation2d(0)))
              : REEF_RED_MIDDLE[nearestReefFaceIndex].transformBy(
                  new Transform2d(0.0, -0.06, new Rotation2d(0)));
    } else if (troughIndex == 4) { // right
      targetPose =
          DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
              ? REEF_BLUE_MIDDLE[nearestReefFaceIndex].transformBy(
                  new Transform2d(0.0, -0.3, new Rotation2d(0)))
              : REEF_RED_MIDDLE[nearestReefFaceIndex].transformBy(
                  new Transform2d(0.0, -0.3, new Rotation2d(0)));
    }

    targetPose = targetPose.transformBy(new Transform2d(L1_REEF_FACE_OFFSET, 0, new Rotation2d(0)));

    DogLog.log("AutoAlignPoseGenerator/TargetPose", targetPose);
    DogLog.log("AutoAlignPoseGenerator/RobotPose", robotPose);
    return targetPose;
  }

  /**
   * Finds nearest reef face position for algae
   *
   * @param robotPose robot pose
   * @param shiftBackFromReefFace true if the target pose should be shifted back from the reef face
   */
  public static Pose2d getNearestAlgaeReefFacePosition(Pose2d robotPose) {

    Pose2d targetPose;
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      targetPose =
          REEF_BLUE_MIDDLE[nearestReefFaceIndex].transformBy(
              new Transform2d(ALGAE_REEF_FACE_OFFSET, 0, new Rotation2d(0)));
    } else {
      targetPose =
          REEF_RED_MIDDLE[nearestReefFaceIndex].transformBy(
              new Transform2d(ALGAE_REEF_FACE_OFFSET, 0, new Rotation2d(0)));
    }

    DogLog.log("AutoAlignPoseGenerator/TargetPose", targetPose);
    DogLog.log("AutoAlignPoseGenerator/RobotPose", robotPose);
    return targetPose;
  }
}
