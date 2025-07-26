package frc.robot.commands;

import static frc.robot.FieldConstants.*;

import choreo.util.ChoreoAllianceFlipUtil;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
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
              : ChoreoAllianceFlipUtil.flip(REEF_BLUE_MIDDLE[i]);

      double distance = robotPose.getTranslation().getDistance(centerFace.getTranslation());

      if (distance < minDistance) {
        minDistance = distance;
        nearestFaceIndex = i;
      }
    }

    Pose2d selectedBranch =
        useLeftBranch ? REEF_BLUE_LEFT[nearestFaceIndex] : REEF_BLUE_RIGHT[nearestFaceIndex];
    Pose2d targetPose =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? selectedBranch
            : ChoreoAllianceFlipUtil.flip(selectedBranch);

    DogLog.log("ClosestFace/TargetPose", targetPose);
    DogLog.log("ClosestFace/RobotPose", robotPose);
    return targetPose;
  }

  /**
   * Finds nearest reef face position for algae
   *
   * @param robotPose robot pose
   * @param moveBack true if the target pose should be moved back from the reef face (used when
   *     algae is already grabbed)
   */
  public static Pose2d getNearestAlgaeReefFacePosition(Pose2d robotPose, boolean moveBack) {
    Pose2d nearestFace = null;
    double minDistance = Double.MAX_VALUE;

    // Find the nearest center face and its index
    for (int i = 0; i < 6; i++) {
      Pose2d centerFace =
          DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
              ? REEF_BLUE_MIDDLE[i]
              : ChoreoAllianceFlipUtil.flip(REEF_BLUE_MIDDLE[i]);

      double distance = robotPose.getTranslation().getDistance(centerFace.getTranslation());

      if (distance < minDistance) {
        minDistance = distance;
        nearestFace = centerFace;
        algaeReefFaceIndex = i;
      }
    }

    DogLog.log("ClosestFace/TargetPose", nearestFace);
    DogLog.log("ClosestFace/RobotPose", robotPose);
    return nearestFace;
  }

  public static int getReefFaceIndex() {
    return algaeReefFaceIndex;
  }
}
