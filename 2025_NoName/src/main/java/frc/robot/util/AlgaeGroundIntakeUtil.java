package frc.robot.util;

import frc.robot.subsystems.algaegroundintake.intakewrist.IntakeWristConstants;

public class AlgaeGroundIntakeUtil {

    public static double convertToTicks(double setpoint){
        return setpoint / IntakeWristConstants.inchesPerRev;
    }

    public static double stateToSetpoint(IntakeWristConstants.States state){
        return convertToTicks(IntakeWristConstants.setpoints[state.getIndex()]);
    }
}
