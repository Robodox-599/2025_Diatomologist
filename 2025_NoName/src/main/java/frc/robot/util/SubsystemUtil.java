package frc.robot.util;

import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.endefector.endefectorrollers.RollersConstants;
import frc.robot.subsystems.endefector.endefectorwrist.WristConstants;

public class SubsystemUtil {

  public static double convertToTicks(double height) {
    return height / ElevatorConstants.inchesPerRev;
  }

  public static double elevatorStateToHeightTicks(ElevatorConstants.ElevatorStates state) {
    return convertToTicks(ElevatorConstants.heights[state.getIndex()]);
  }

  public static double elevatorStateToHeightInches(ElevatorConstants.ElevatorStates state) {
    return ElevatorConstants.heights[state.getIndex()];
  }

  public static double rollersStateToVelocity(RollersConstants.EndefectorRollerStates state) {
    return RollersConstants.velocitys[state.getIndex()];
  }

  public static double climbStateToHeight(ClimbConstants.ClimbStates state) {
    return (ClimbConstants.setpoint[state.getIndex()]);
  }

  public static double wristStateToSetpoint(WristConstants.WristStates state) {
    return (WristConstants.setpoints[state.getIndex()]);
  }
}
