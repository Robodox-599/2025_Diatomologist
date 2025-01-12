package frc.robot.Subsystems.Elevator;

import edu.wpi.first.units.Units;

public final class ElevatorConstants {
    /* CHANGE LATER */

    public static final int leaderMotorID = 20;
    public static final String leaderMotorCANbus = "temp";
    public static final int followerMotorID = 69;
    public static final String followerMotorCANbus = "changelater";
    public static final boolean followerInverted = true;

    public static final int hallEffectDioPort = 0;
    public static final double gearRatio = 10.0;
    public static final double drumCircumferenceInches = 1.8;
    public static final double countsPerRevolution = 2048.0;
    public static final double inchesPerCount = (drumCircumferenceInches) / (countsPerRevolution * gearRatio);
    public static final double drumRadiusMeters = Units.Inches.of(2).magnitude();
    
    public static final double maxVelocityInchesPerSec = 60.0;
    public static final double maxAccelerationInchesPerSecSQ = 120.0;
    
    /* Position Setpoints (in inches) */
    public static final double elevatorLowerLimit = 0.0;
    public static final double elevatorUpperLimit = 48.0;
    public static final double homePositionOffset = 1.0;
    
    public static final double levelOneHeight = 0.0;
    public static final double levelTwoHeight = 12.0;
    public static final double levelThreeHeight = 24.0;
    public static final double levelFourHeight = 36.0;
    
    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kF = 0.0;

    public static final double simkP = 0.1;
    public static final double simkI = 0.0;
    public static final double simkD = 0.0;
    public static final double simkF = 0.0;

    public static final double supplyCurrentLimitAmps = 0.1;
    public static final double statorCurrentLimitAmps = 0.1;
    
    public static final double PositionToleranceInches = 0.5;
    public static final double velocityToleranceInchesPerSec = 0.5;

    public static final double elevatorMOI = 0.025;
    public static final double nominal_voltage = 12.0;
}