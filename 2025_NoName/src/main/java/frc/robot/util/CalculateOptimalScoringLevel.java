package frc.robot.util;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;

public class CalculateOptimalScoringLevel {
  private static final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  private static final NetworkTable coralLevelTable =
      inst.getTable("/SmartDashboard/CoralPosition");

  public static boolean calculateIfOptimalScoringBranchIsLeft() {
    return calculateOptimalScoringLevel(true) >= calculateOptimalScoringLevel(false);
  }

  public static int calculateOptimalScoringLevel(boolean isLeftBranch) {
    int nearestFaceIndex = AutoAlignPoseGenerator.getNearestReefFaceIndex();

    String branch = getBranch(nearestFaceIndex, isLeftBranch);
    boolean isL2CoralScored = coralLevelTable.getEntry("L2-" + branch).getBoolean(false);
    boolean isL3CoralScored = coralLevelTable.getEntry("L3-" + branch).getBoolean(false);
    boolean isL4CoralScored = coralLevelTable.getEntry("L4-" + branch).getBoolean(false);

    boolean isRPMode = coralLevelTable.getEntry("RPMode").getBoolean(false);

    int totalL4Coral = getTotalL4Coral();
    int totalL3Coral = getTotalL3Coral();
    int totalL2Coral = getTotalL2Coral();

    DogLog.log("CalculateOptimalScoringLevel/NearestFaceIndex", nearestFaceIndex);
    DogLog.log("CalculateOptimalScoringLevel/Branch", branch);
    DogLog.log("CalculateOptimalScoringLevel/isRPMode", isRPMode);
    DogLog.log("CalculateOptimalScoringLevel/Branch/L2", isL2CoralScored);
    DogLog.log("CalculateOptimalScoringLevel/Total/L2", totalL2Coral);
    DogLog.log("CalculateOptimalScoringLevel/Branch/L3", isL3CoralScored);
    DogLog.log("CalculateOptimalScoringLevel/Total/L3", totalL3Coral);
    DogLog.log("CalculateOptimalScoringLevel/Branch/L4", isL4CoralScored);
    DogLog.log("CalculateOptimalScoringLevel/Total/L4", totalL4Coral);

    if (isRPMode) {
      if (totalL4Coral < 6 && !isL4CoralScored) {
        return 4;
      } else if (totalL3Coral < 6 && !isL3CoralScored) {
        return 3;
      } else if (totalL2Coral < 6 && !isL2CoralScored) {
        return 2;
      }
    }

    if (!isL4CoralScored) {
      return 4;
    } else if (!isL3CoralScored) {
      return 3;
    } else if (!isL2CoralScored) {
      return 2;
    } else {
      return 1;
    }
  }

  private static String getBranch(int nearestFaceIndex, boolean isLeftBranch) {
    switch (nearestFaceIndex) {
      default:
      case 0:
        return isLeftBranch ? "A" : "B";
      case 1:
        return isLeftBranch ? "C" : "D";
      case 2:
        return isLeftBranch ? "E" : "F";
      case 3:
        return isLeftBranch ? "G" : "H";
      case 4:
        return isLeftBranch ? "I" : "J";
      case 5:
        return isLeftBranch ? "K" : "L";
    }
  }

  private static int getTotalL4Coral() {
    int totalL4Coral = 0;
    for (int i = 0; i < 12; i++) {
      totalL4Coral +=
          coralLevelTable.getEntry("L4-" + String.valueOf((char) ('A' + i))).getBoolean(false)
              ? 1
              : 0;
    }
    return totalL4Coral;
  }

  private static int getTotalL3Coral() {
    int totalL3Coral = 0;
    for (int i = 0; i < 12; i++) {
      totalL3Coral +=
          coralLevelTable.getEntry("L3-" + String.valueOf((char) ('A' + i))).getBoolean(false)
              ? 1
              : 0;
    }
    return totalL3Coral;
  }

  private static int getTotalL2Coral() {
    int totalL2Coral = 0;
    for (int i = 0; i < 12; i++) {
      totalL2Coral +=
          coralLevelTable.getEntry("L2-" + String.valueOf((char) ('A' + i))).getBoolean(false)
              ? 1
              : 0;
    }
    return totalL2Coral;
  }

  public static void addCoral(int level, boolean isLeftBranch) {
    int nearestFaceIndex = AutoAlignPoseGenerator.getNearestReefFaceIndex();
    String branch = getBranch(nearestFaceIndex, isLeftBranch);
    coralLevelTable.putValue("L" + level + "-" + branch, NetworkTableValue.makeBoolean(true));
  }
}
