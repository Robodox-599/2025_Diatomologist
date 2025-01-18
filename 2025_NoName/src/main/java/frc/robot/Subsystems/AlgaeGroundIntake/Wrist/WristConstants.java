package frc.robot.subsystems.algaegroundintake.wrist;

import edu.wpi.first.math.util.Units;

public class WristConstants {
    
    public static final int wristMotorID = 18; 
    public static final String wristMotorCANBus = "rio"; //change later
    
    public static final double intakeWristPositionTolerance =
        Units.degreesToRadians(1);  
    public static final double intakeWristVelocityTolerance = 0.5;  
    
    public static final double intakeWristMaxAngle =
        Units.degreesToRadians(0.0);  
    public static final double intakeWristMinAngle =
        Units.degreesToRadians(80);  
    
    public static final double kWristRetractVal = 0.0;  
    public static final double kWristExtendVal = Units.degreesToRadians(75.0);  
    public static final int wristExtendSlot = 0;
    public static final double wristExtendKP = 7;
    public static final double wristExtendKI = 0;
    public static final double wristExtendKD = 0.0;
    public static final double wristExtendKS = 0.0;
    public static final double kWristFeedForward = -0.4; 
    
    public static final double AbsWristP = -7;
    public static final double AbsWristI = 0;
    public static final double AbsWristD = 0;
    public static final double AbsWristFeedForward = -0.8; 
    public static final int wristRetractSlot = 1;
    public static final double wristRetractKP = 8;
    public static final double wristRetractKI = 0.0;
    public static final double wristRetractKD = 0.2;
    
    public static final double maxWristVelocity = 120;
    public static final double maxWristAccel = 240;
    
    public static class IntakeWristSimConstants {
      public static final double[] kPivotSimPID = {15, 0, 0, 0};  
    
      public static final int kMotorPort = 2;  
      public static final int kEncoderAChannel = 4;  
      public static final int kEncoderBChannel = 5;  
    
     
      public static final double kDefaultArmSetpointDegrees =
          Units.degreesToRadians(75.0);  
    
      
          
      public static final double kArmEncoderDistPerPulse = 1 / 4096;  
    
      public static final double kArmReduction = 200; 
      public static final double kArmMass = 10.0;
      public static final double kArmLength = Units.inchesToMeters(20);  
      public static final double kMinAngleRads = Units.degreesToRadians(0);  
      public static final double kMaxAngleRads = Units.degreesToRadians(180);  
    }

}

    
      