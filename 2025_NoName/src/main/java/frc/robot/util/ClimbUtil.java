package frc.robot.util;

import frc.robot.subsystems.climb.ClimbConstants;

public class ClimbUtil {

  public static double convertToTicks(double height) {
    return height / ClimbConstants.inchesPerRev;
  }

  public static double stateToHeight(ClimbConstants.ClimbStates state) {
    return convertToTicks(ClimbConstants.heights[state.getIndex()]);
  }
}
