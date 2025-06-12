package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SafetyChecker;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endefector.endefectorrollers.Rollers;
import frc.robot.subsystems.endefector.endefectorwrist.Wrist;
import frc.robot.subsystems.leds.LEDs;

public class Superstructure extends SubsystemBase {
  private final CommandSwerveDrivetrain drivetrain;
  private final Elevator elevator;
  private final Wrist wrist;
  private final Rollers rollers;
  private final LEDs leds;
  private final SafetyChecker safetyChecker;

  private CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;
  private WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
  private WantedSuperState nextSuperState = WantedSuperState.STOPPED;

  public enum CurrentSuperState {
    INTAKING_CORAL_STATION,
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
    SCORING_CORAL,
    SCORING_ALGAE,
    STOPPED,
  }

  public enum WantedSuperState {
    INTAKING_CORAL_STATION,
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
    SCORING_CORAL,
    SCORING_ALGAE,
    STOPPED,
  }

  public Superstructure(
      CommandSwerveDrivetrain drivetrain,
      Elevator elevator,
      Wrist wrist,
      Rollers rollers,
      // Climb climb,
      LEDs LEDs,
      SafetyChecker safetyChecker) {
    this.drivetrain = drivetrain;
    this.elevator = elevator;
    this.wrist = wrist;
    this.rollers = rollers;
    this.leds = LEDs;
    this.safetyChecker = safetyChecker;
  }

  @Override
  public void periodic() {
    elevator.updateInputs();
    rollers.updateInputs();
    wrist.updateInputs();
    leds.updateInputs();
    currentSuperState = handleStateTransitions();
    applyStates();

    DogLog.log("Superstructure/NextSuperState", nextSuperState);
    DogLog.log("Superstructure/WantedSuperState", wantedSuperState);
    DogLog.log("Superstructure/CurrentSuperState", currentSuperState);
  }

  private CurrentSuperState handleStateTransitions() {
    switch (wantedSuperState) {
      case INTAKING_CORAL_STATION:
        if (rollers.isCoralDetected()) {
          currentSuperState = CurrentSuperState.POSITION_PREPARED;
          wantedSuperState = WantedSuperState.POSITION_PREPARED;
        } else {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
        }
        break;
      case INTAKING_ALGAE_GROUND:
        if (rollers.isAlgaeDetected()) {
          currentSuperState = CurrentSuperState.POSITION_ALGAE_PROCESSOR;
          wantedSuperState = WantedSuperState.POSITION_ALGAE_PROCESSOR;
        } else {
          currentSuperState = CurrentSuperState.INTAKING_ALGAE_GROUND;
        }
        break;
      case INTAKING_ALGAE_L2:
        if (rollers.isAlgaeDetected()) {
          currentSuperState = CurrentSuperState.POSITION_ALGAE_PROCESSOR;
          wantedSuperState = WantedSuperState.POSITION_ALGAE_PROCESSOR;
        } else {
          currentSuperState = CurrentSuperState.INTAKING_ALGAE_L2;
        }
        break;
      case INTAKING_ALGAE_L3:
        if (rollers.isAlgaeDetected()) {
          currentSuperState = CurrentSuperState.POSITION_ALGAE_PROCESSOR;
          wantedSuperState = WantedSuperState.POSITION_ALGAE_PROCESSOR;
        } else {
          currentSuperState = CurrentSuperState.INTAKING_ALGAE_L3;
        }
        break;
      case POSITION_PREPARED:
        currentSuperState = CurrentSuperState.POSITION_PREPARED;
        break;
      case POSITION_CORAL_L1:
          currentSuperState = CurrentSuperState.POSITION_CORAL_L1;
        break;
      case POSITION_CORAL_L2:
          currentSuperState = CurrentSuperState.POSITION_CORAL_L2;
        break;
      case POSITION_CORAL_L3:
          currentSuperState = CurrentSuperState.POSITION_CORAL_L3;
        break;
      case POSITION_CORAL_L4:
          currentSuperState = CurrentSuperState.POSITION_CORAL_L4;
        break;
      case POSITION_ALGAE_PROCESSOR:
          currentSuperState = CurrentSuperState.POSITION_ALGAE_PROCESSOR;
        break;
      case POSITION_ALGAE_BARGE:
          currentSuperState = CurrentSuperState.POSITION_ALGAE_BARGE;
        break;
      case SCORING_CORAL:
        if (!rollers.isCoralDetected()) {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
          wantedSuperState = WantedSuperState.INTAKING_CORAL_STATION;
        } else {
          currentSuperState = CurrentSuperState.SCORING_CORAL;
        }
        break;
      case SCORING_ALGAE:
        if (!rollers.isAlgaeDetected()) {
          currentSuperState = CurrentSuperState.POSITION_PREPARED;
          wantedSuperState = WantedSuperState.POSITION_PREPARED;
        } else {
          currentSuperState = CurrentSuperState.SCORING_ALGAE;
        }
        break;
      case STOPPED:
        currentSuperState = CurrentSuperState.STOPPED;
        break;
      default:
        currentSuperState = CurrentSuperState.STOPPED;
        break;
    }
    return currentSuperState;
  }

  private void applyStates() {
    switch (currentSuperState) {
      case INTAKING_CORAL_STATION:
        intakeCoralStation();
        break;
      case INTAKING_ALGAE_GROUND:
        intakeAlgaeGround();
        break;
      case INTAKING_ALGAE_L2:
        intakeAlgaeL2();
        break;
      case INTAKING_ALGAE_L3:
        intakeAlgaeL3();
        break;
      case POSITION_PREPARED:
        prepare();
        break;
      case POSITION_CORAL_L1:
        positionToCoralL1();
        break;
      case POSITION_CORAL_L2:
        positionToCoralL2();
        break;
      case POSITION_CORAL_L3:
        positionToCoralL3();
        break;
      case POSITION_CORAL_L4:
        positionToCoralL4();
        break;
      case POSITION_ALGAE_PROCESSOR:
        positionToAlgaeProcessor();
        break;
      case POSITION_ALGAE_BARGE:
        positionToAlgaeBarge();
        break;
      case SCORING_CORAL:
        scoreCoral();
        break;
      case SCORING_ALGAE:
        scoreAlgae();
        break;
      case STOPPED:
        stop();
        break;
      default:
        stop();
        break;
    }
  }

  private void intakeCoralStation() {
    elevator.setWantedState(Elevator.WantedState.INTAKING_CORAL_STATION);
    rollers.setWantedState(Rollers.WantedState.INTAKING_CORAL_STATION);
    wrist.setWantedState(Wrist.WantedState.INTAKING_CORAL_STATION);
    leds.setCurrentState(LEDs.CurrentState.INTAKING_CORAL_STATION);
  }

  private void intakeAlgaeGround() {
    elevator.setWantedState(Elevator.WantedState.INTAKING_ALGAE_GROUND);
    rollers.setWantedState(Rollers.WantedState.INTAKING_ALGAE);
    wrist.setWantedState(Wrist.WantedState.INTAKING_ALGAE_GROUND);
    leds.setCurrentState(LEDs.CurrentState.INTAKING_ALGAE_GROUND);
  }

  private void intakeAlgaeL2() {
    elevator.setWantedState(Elevator.WantedState.INTAKING_ALGAE_L2);
    rollers.setWantedState(Rollers.WantedState.INTAKING_ALGAE);
    wrist.setWantedState(Wrist.WantedState.INTAKING_ALGAE_REEF);
    leds.setCurrentState(LEDs.CurrentState.INTAKING_ALGAE_L2);
  }

  private void intakeAlgaeL3() {
    elevator.setWantedState(Elevator.WantedState.INTAKING_ALGAE_L3);
    rollers.setWantedState(Rollers.WantedState.INTAKING_ALGAE);
    wrist.setWantedState(Wrist.WantedState.INTAKING_ALGAE_REEF);
    leds.setCurrentState(LEDs.CurrentState.INTAKING_ALGAE_L3);
  }

  private void prepare() {
    elevator.setWantedState(Elevator.WantedState.POSITION_PREPARED);
    rollers.setWantedState(Rollers.WantedState.STOPPED);
    wrist.setWantedState(Wrist.WantedState.POSITION_PREPARED);
    leds.setCurrentState(LEDs.CurrentState.POSITION_PREPARED);
  }

  private void positionToCoralL1() {
    elevator.setWantedState(Elevator.WantedState.POSITION_CORAL_L1);
    wrist.setWantedState(Wrist.WantedState.SCORING_CORAL);
    leds.setCurrentState(LEDs.CurrentState.POSITION_CORAL_L1);
  }

  private void positionToCoralL2() {
    elevator.setWantedState(Elevator.WantedState.POSITION_CORAL_L2);
    wrist.setWantedState(Wrist.WantedState.SCORING_CORAL);
    leds.setCurrentState(LEDs.CurrentState.POSITION_CORAL_L2);
  }

  private void positionToCoralL3() {
    elevator.setWantedState(Elevator.WantedState.POSITION_CORAL_L3);
    wrist.setWantedState(Wrist.WantedState.SCORING_CORAL);
    leds.setCurrentState(LEDs.CurrentState.POSITION_CORAL_L3);
  }

  private void positionToCoralL4() {
    elevator.setWantedState(Elevator.WantedState.POSITION_CORAL_L4);
    wrist.setWantedState(Wrist.WantedState.SCORING_CORAL);
    leds.setCurrentState(LEDs.CurrentState.POSITION_CORAL_L4);
  }

  private void positionToAlgaeProcessor() {
    elevator.setWantedState(Elevator.WantedState.POSITION_ALGAE_PROCESSOR);
    wrist.setWantedState(Wrist.WantedState.SCORING_ALGAE);
    leds.setCurrentState(LEDs.CurrentState.SCORING_ALGAE);
  }

  private void positionToAlgaeBarge() {
    elevator.setWantedState(Elevator.WantedState.POSITION_ALGAE_BARGE);
    wrist.setWantedState(Wrist.WantedState.SCORING_ALGAE);
    leds.setCurrentState(LEDs.CurrentState.SCORING_ALGAE);
  }

  private void scoreCoral() {
    rollers.setWantedState(Rollers.WantedState.SCORING_CORAL);
    leds.setCurrentState(LEDs.CurrentState.SCORING_CORAL);
  }

  private void scoreAlgae() {
    rollers.setWantedState(Rollers.WantedState.SCORING_ALGAE);
    leds.setCurrentState(LEDs.CurrentState.SCORING_ALGAE);
  }

  private void stop() {
    elevator.setWantedState(Elevator.WantedState.STOPPED);
    rollers.setWantedState(Rollers.WantedState.STOPPED);
    wrist.setWantedState(Wrist.WantedState.STOPPED);
    leds.setCurrentState(LEDs.CurrentState.NO_STATE);
  }

  public Command setNextSuperStateCommand(WantedSuperState nextState) {
    return this.runOnce(() -> setNextSuperState(nextState));
  }
  
  private void setNextSuperState(WantedSuperState state) {
    nextSuperState = state;
  }

  public Command updateWantedSuperStateCommand() {
    return this.runOnce(() -> updateWantedSuperState());
  }

  private void updateWantedSuperState() {
    wantedSuperState = nextSuperState;
    nextSuperState = WantedSuperState.STOPPED;
  }

  public Command setWantedSuperStateCommand(WantedSuperState wantedState) {
    return this.runOnce(() -> setWantedSuperState(wantedState));
  }

  private void setWantedSuperState(WantedSuperState state) {
    wantedSuperState = state;
    nextSuperState = WantedSuperState.STOPPED;
  }

  private boolean isAtWantedState() {
    return (wantedSuperState.name().equals(currentSuperState.name())) && safetyChecker.isAtSetpoints();
  }

  public Command zeroGyroCommand() {
    return this.runOnce(() -> drivetrain.zeroGyro());
  }
}
