package frc.robot.util;

import frc.robot.subsystems.endefector.wrist.WristConstants;

public class EndefectorUtil {

    public static double convertToTicks(double setpoint){
        return setpoint / WristConstants.inchesPerRev;
    }

    public static double stateToSetpoint(WristConstants.WristStates state){
        return convertToTicks(WristConstants.setpoints[state.getIndex()]);
    }
}
