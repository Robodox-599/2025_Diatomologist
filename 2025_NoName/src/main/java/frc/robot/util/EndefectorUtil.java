package frc.robot.util;

import frc.robot.subsystems.endefector.endefectorrollers.RollersConstants;
import frc.robot.subsystems.endefector.endefectorwrist.WristConstants;

public class EndefectorUtil {

  public static double convertToTicks(double setpoint) {
    return setpoint / WristConstants.inchesPerRev;
  }

  public static double stateToSetpoint(WristConstants.WristStates state) {
    return convertToTicks(WristConstants.setpoints[state.getIndex()]);
  }

  public static double stateToVelocity(RollersConstants.EndefectorRollerStates state) {
    return convertToTicks(RollersConstants.velocitys[state.getIndex()]);
  }
}
