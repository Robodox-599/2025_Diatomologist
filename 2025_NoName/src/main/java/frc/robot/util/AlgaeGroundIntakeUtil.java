package frc.robot.util;

import frc.robot.subsystems.algaegroundintake.intakeWrist.IntakeWristConstants;

public class AlgaeGroundIntakeUtil {

  public static double convertToTicks(double setpoint) {
    return setpoint / IntakeWristConstants.inchesPerRev;
  }

  public static double stateToSetpoint(IntakeWristConstants.AlgaeStates state) {
    return convertToTicks(IntakeWristConstants.setpoints[state.getIndex()]);
  }
}
