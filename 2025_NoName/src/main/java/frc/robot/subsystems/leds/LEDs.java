package frc.robot.subsystems.leds;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  private final LEDsIO io;
  private CurrentState currentState = CurrentState.NO_STATE;

  public LEDs(
      LEDsIO
          io) { // TODO: post integration, add other subsystems here so we can switch from running a
    // command to using the periodic to grab subsystem states and update LEDs that way.
    this.io = io;
  }

  public enum CurrentState {
    INTAKING_CORAL_STATION,
    INTAKING_ALGAE_GROUND,
    INTAKING_ALGAE_L2,
    INTAKING_ALGAE_L3,
    PREPARED,
    SCORING_CORAL_L1,
    SCORING_CORAL_L2,
    SCORING_CORAL_L3,
    SCORING_CORAL_L4,
    SCORING_ALGAE_PROCESSOR,
    SCORING_ALGAE_BARGE,
    NO_STATE,
  }

  @Override
  public void periodic() {
    io.updateInputs();
    disableAction();
    applyStates();
    DogLog.log("Wrist/CurrentState", currentState);
  }

  public void applyStates() {}

  public void setCurrentState(CurrentState currentState) {
    this.currentState = currentState;
  }

  private void disableAction() {
    if (DriverStation.isDisabled()) {
      setCurrentState(CurrentState.NO_STATE);
    }
  }
}
