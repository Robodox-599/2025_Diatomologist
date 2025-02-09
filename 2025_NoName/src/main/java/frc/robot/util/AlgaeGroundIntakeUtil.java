package frc.robot.util;

import frc.robot.subsystems.algaegroundintake.intakeRollers.IntakeRollersConstants;
import frc.robot.subsystems.algaegroundintake.intakewrist.IntakeWristConstants;

public class AlgaeGroundIntakeUtil {

  public static double convertToTicks(double setpoint) {
    return setpoint / IntakeWristConstants.inchesPerRev;
  }

  public static double stateToSetpoint(IntakeWristConstants.AlgaeStates state) {
    return convertToTicks(IntakeWristConstants.setpoints[state.getIndex()]);
  }

  public static double stateToVelocity(IntakeRollersConstants.AlgaeRollerStates state) {
    return convertToTicks(IntakeRollersConstants.velocitys[state.getIndex()]);
  }
}
