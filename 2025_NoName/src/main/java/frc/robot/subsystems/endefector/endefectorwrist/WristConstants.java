package frc.robot.subsystems.endefector.endefectorwrist;

import frc.robot.Constants;

public class WristConstants {
  // motor info
  public static final int wristMotorID = 15;
  public static final int cancoderID = 17;
  public static final String wristMotorCANBus = "rio";
  public static final double gearRatio = 35.0;
  public static final double wristMOI = 0.04;
  public static final double wristPositionTolerance = 0.01; // rotations

  // current limit stuff
  public static final boolean EnableCurrentLimit = true;
  public static final int ContinousCurrentLimit = 50;
  public static final int PeakCurrentLimit = 50;
  public static final double PeakCurrentDuration = 0.1;

  public static final double realkP = 38;
  public static final double realkI = 0.0;
  public static final double realkD = 0.0;
  public static final double realkS = 0.075;
  public static final double realkV = Constants.kMotors.kKrakenX60Foc.kV * gearRatio;
  public static final double realkGNoCoral = 0.42;
  public static final double realKgWithCoral = 0.52;

  public static final double maxWristVelocityNoCoral = (12 - realkGNoCoral - realkS) / realkV;
  public static final double maxWristVelocityWithCoral = (12 - realKgWithCoral - realkS) / realkV;
  public static final double maxWristAccelerationNoCoral = maxWristVelocityNoCoral / 0.4; // set update to 150hz, set max accel to 5 * velocity, find the max accel, and multiply by 0.8 
  public static final double maxWristAccelerationWithCoral = maxWristVelocityWithCoral / 0.4;

  // sim stuff
  public static final double simkP = 6.9;
  public static final double simkI = 0.5;
  public static final double simkD = 2.25;
  public static final double simkV = 0.0;
  public static final double simkS = 0.0;
  public static final double simVelocityConstant = 0.2;

  public static final double cancoderOffset = -0.14892578125;

  // setpoints
  public static final double wristMinAngle = -0.35;
  public static final double wristMaxAngle = 0.1;

  // wrist state stuff
  public static enum WristStates {
    INTAKING_CORAL_STATION(0),
    INTAKING_ALGAE_GROUND(1),
    INTAKING_ALGAE_LOLLIPOP(2),
    INTAKING_ALGAE_REEF(3),
    POSITION_PREPARED(4),
    POSITION_TROUGH(5),
    POSITION_BRANCH_L2(6),
    POSITION_BRANCH_L3(7),
    POSITION_BRANCH_L4(8),
    SCORING_ALGAE_BARGE(9);

    private final int index;

    WristStates(int index) {
      this.index = index;
    }

    public int getIndex() {
      return index;
    }
  }

  public static final double[] setpoints = {
    -0.305, // INTAKING CORAL STATION
    -0.1, // INTAKING ALGAE GROUND
    -0.2, // INTAKING ALGAE LOLLIPOP
    -0.1, // INTAKING ALGAE REEF
    -0.226, // POSITION PREPARED
    0.0, // POSITION TROUGH
    0.11, // POSITION BRANCH L2
    0.11, // POSITION BRANCH L3
    -0.226, // POSITION BRANCH L4
    -0.226 // SCORING ALGAE BARGE
  };
}
