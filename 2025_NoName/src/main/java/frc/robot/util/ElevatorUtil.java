package frc.robot.util;

import frc.robot.subsystems.elevator.ElevatorConstants;

public class ElevatorUtil{

    public static double convertToTicks(double height){
        return height / ElevatorConstants.inchesPerCount;
    }

    public static double stateToHeight(ElevatorConstants.ElevatorStates state){
        return convertToTicks(ElevatorConstants.heights[state.getIndex()]);
    }
    
}