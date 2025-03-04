package frc.robot.util;

import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.endefector.endefectorwrist.WristConstants;

public class SubsystemUtil {

  public static double convertToTicks(double height) {
    return height / ElevatorConstants.inchesPerRev;
  }

  public static double elevatorStateToHeight(ElevatorConstants.ElevatorStates state) {
    return convertToTicks(ElevatorConstants.heights[state.getIndex()]);
  }

  public static double climbStateToHeight(ClimbConstants.ClimbStates state) {
    return convertToTicks(ClimbConstants.setpoint[state.getIndex()]);
  }

  public static double wristStateToSetpoint(WristConstants.WristStates state) {
    return convertToTicks(WristConstants.setpoints[state.getIndex()]);
  }
}
