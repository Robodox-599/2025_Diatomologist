package frc.robot.subsystems.leds;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.Tracer;

public class LEDs {
  private final LEDsIO io;
  private CurrentState currentState = CurrentState.STOPPED;

  public LEDs(
      LEDsIO
          io) { // TODO: post integration, add other subsystems here so we can switch from running a
    // command to using the periodic to grab subsystem states and update LEDs that way.
    this.io = io;
  }

  public enum CurrentState {
    INTAKING_CORAL_STATION,
    ENSURING_CORAL,
    INTAKING_ALGAE_GROUND,
    INTAKING_ALGAE_L2,
    INTAKING_ALGAE_L3,
    POSITION_PREPARED,
    POSITION_CORAL_L1,
    POSITION_CORAL_L2,
    POSITION_CORAL_L3,
    POSITION_CORAL_L4,
    POSITION_ALGAE_PROCESSOR,
    POSITION_ALGAE_BARGE,
    POSITION_CLIMB_PREPARED,
    SCORING_CORAL,
    SCORING_ALGAE,
    CLIMBING,
    STOPPED,
  }

  public void updateInputs() {
    if (DriverStation.isDisabled()) {
      currentState = CurrentState.STOPPED;
    }
    // Tracer.traceFunc("UpdateIO", io::updateInputs);
    DogLog.log("LEDs/CurrentState", currentState);
  }

  public void applyStates() {
    switch (currentState) {
      case INTAKING_CORAL_STATION:
        io.LEDsIntakingCoralStation();
        break;
      case ENSURING_CORAL:
        io.LEDsEnsuringCoral();
        break;
      case INTAKING_ALGAE_GROUND:
        io.LEDsIntakingAlgae();
        break;
      case INTAKING_ALGAE_L2:
        io.LEDsIntakingAlgae();
        break;
      case INTAKING_ALGAE_L3:
        io.LEDsIntakingAlgae();
        break;
      case POSITION_PREPARED:
        io.LEDsPositionPrepared();
        break;
      case POSITION_CORAL_L1:
        io.LEDsPositionCoral();
        break;
      case POSITION_CORAL_L2:
        io.LEDsPositionCoral();
        break;
      case POSITION_CORAL_L3:
        io.LEDsPositionCoral();
        break;
      case POSITION_CORAL_L4:
        io.LEDsPositionCoral();
        break;
      case POSITION_ALGAE_PROCESSOR:
        io.LEDsPositionAlgae();
        break;
      case POSITION_ALGAE_BARGE:
        io.LEDsPositionAlgae();
        break;
      case POSITION_CLIMB_PREPARED:
        io.LEDsPositionClimbPrepared();
        break;
      case SCORING_CORAL:
        io.LEDsScoringGamePiece();
        break;
      case SCORING_ALGAE:
        io.LEDsScoringGamePiece();
        break;
      case CLIMBING:
        io.LEDsClimbing();
        break;
      case STOPPED:
        io.LEDsStopped();
        break;
      default:
        io.LEDsStopped();
        break;
    }
  }

  public void setCurrentState(CurrentState currentState) {
    this.currentState = currentState;
    applyStates();
  }
}
