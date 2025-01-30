package frc.robot.util;

import static frc.robot.subsystems.endefector.wrist.WristConstants.*;

public class WristUtil{

    public static double convertToTicks(double height){
        return height / inchesPerRev;
    }

    public static double stateToHeight(WristStates state){
        return convertToTicks(setpoints[state.getIndex()]);
    }
    
}