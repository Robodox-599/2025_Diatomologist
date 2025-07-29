package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.SafetyChecker;
import frc.robot.commands.AutoAlignPoseGenerator;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endefector.endefectorrollers.Rollers;
import frc.robot.subsystems.endefector.endefectorwrist.Wrist;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.vision2.Vision;

public class Superstructure extends SubsystemBase {
  private final CommandSwerveDrivetrain drivetrain;
  private final Elevator elevator;
  private final Wrist wrist;
  private final Rollers rollers;
  private final LEDs leds;
  // private final Climb climb;
  private final Vision vision;
  private final SafetyChecker safetyChecker;
  private final CommandXboxController driver;
  private final CommandXboxController operator;

  private CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;
  private WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
  private WantedSuperState queuedSuperState = WantedSuperState.STOPPED;
  public AutomationLevel automationLevel = AutomationLevel.AUTO_ACTION;

  public enum CurrentSuperState {
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
    AUTO_ALIGN_LEFT_BRANCH,
    AUTO_ALIGN_RIGHT_BRANCH,
    AUTO_ALIGN_MIDDLE_ALGAE,
    AUTO_ALIGN_MIDDLE_BACK_AND_POSITION_ALGAE_PROCESSOR,
    POSITION_ALGAE_PROCESSOR,
    POSITION_ALGAE_BARGE,
    POSITION_CLIMB_PREPARED,
    SCORING_CORAL,
    SCORING_ALGAE,
    CLIMBING,
    STOPPED,
  }

  public enum WantedSuperState {
    INTAKING_CORAL_STATION,
    ENSURING_CORAL,
    INTAKING_ALGAE_GROUND,
    INTAKING_ALGAE_L2,
    INTAKING_ALGAE_L3,
    AUTO_INTAKE_ALGAE,
    POSITION_PREPARED,
    POSITION_CORAL_L1,
    POSITION_CORAL_L2,
    POSITION_CORAL_L3,
    POSITION_CORAL_L4,
    AUTO_ALIGN_LEFT_BRANCH,
    AUTO_ALIGN_RIGHT_BRANCH,
    AUTO_ALIGN_MIDDLE_ALGAE,
    AUTO_SCORE_L1,
    AUTO_SCORE_L1_LEFT,
    AUTO_SCORE_L1_RIGHT,
    AUTO_SCORE_L2,
    AUTO_SCORE_L2_LEFT,
    AUTO_SCORE_L2_RIGHT,
    AUTO_SCORE_L3,
    AUTO_SCORE_L3_LEFT,
    AUTO_SCORE_L3_RIGHT,
    AUTO_SCORE_L4,
    AUTO_SCORE_L4_LEFT,
    AUTO_SCORE_L4_RIGHT,
    POSITION_ALGAE_PROCESSOR,
    POSITION_ALGAE_BARGE,
    POSITION_CLIMB_PREPARED,
    SCORING_CORAL,
    SCORING_ALGAE,
    CLIMBING,
    STOPPED,
    NO_STATE,
  }

  public enum AutomationLevel {
    MANUAL,
    AUTO_ACTION,
  }

  public Superstructure(
      CommandSwerveDrivetrain drivetrain,
      Elevator elevator,
      Wrist wrist,
      Rollers rollers,
      LEDs LEDs,
      // Climb climb,
      Vision vision,
      SafetyChecker safetyChecker,
      CommandXboxController driver,
      CommandXboxController operator) {
    this.drivetrain = drivetrain;
    this.elevator = elevator;
    this.wrist = wrist;
    this.rollers = rollers;
    this.leds = LEDs;
    // this.climb = climb;
    this.vision = vision;
    this.safetyChecker = safetyChecker;
    this.driver = driver;
    this.operator = operator;
  }

  @Override
  public void periodic() {
    drivetrain.updateInputs();
    elevator.updateInputs();
    rollers.updateInputs();
    wrist.updateInputs();
    leds.updateInputs();
    // climb.updateInputs();
    vision.update();
    currentSuperState = handleStateTransitions();
    applyStates();

    DogLog.log("Superstructure/CurrentSuperState", currentSuperState);
    DogLog.log("Superstructure/WantedSuperState", wantedSuperState);
    DogLog.log("Superstructure/QueuedSuperState", queuedSuperState);
    DogLog.log("Superstructure/AutomationLevel", automationLevel);
  }

  private CurrentSuperState handleStateTransitions() {
    switch (wantedSuperState) {
      case INTAKING_CORAL_STATION:
        if (rollers.isCoralDetected()) {
          currentSuperState = CurrentSuperState.ENSURING_CORAL;
          wantedSuperState = WantedSuperState.ENSURING_CORAL;
        } else {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
        }
        break;
      case ENSURING_CORAL:
        if (rollers.isCoralEnsured()) {
          currentSuperState = CurrentSuperState.POSITION_PREPARED;
          wantedSuperState = WantedSuperState.POSITION_PREPARED;
        } else if (!rollers.isCoralDetected()) {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
          wantedSuperState = WantedSuperState.INTAKING_CORAL_STATION;
        } else {
          currentSuperState = CurrentSuperState.ENSURING_CORAL;
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
      case AUTO_INTAKE_ALGAE:
        if (rollers.isAlgaeDetected()
            && drivetrain.isAtDriveToPointSetpoints()
            && currentSuperState
                == CurrentSuperState.AUTO_ALIGN_MIDDLE_BACK_AND_POSITION_ALGAE_PROCESSOR) {
          wantedSuperState = WantedSuperState.POSITION_ALGAE_PROCESSOR;
          currentSuperState = CurrentSuperState.POSITION_ALGAE_PROCESSOR;
          break;
        } else if (rollers.isAlgaeDetected()) {
          currentSuperState = CurrentSuperState.AUTO_ALIGN_MIDDLE_BACK_AND_POSITION_ALGAE_PROCESSOR;
          break;
        } else if (drivetrain.isAtDriveToPointSetpoints()
            && (currentSuperState == CurrentSuperState.AUTO_ALIGN_MIDDLE_ALGAE
                || currentSuperState == CurrentSuperState.INTAKING_ALGAE_L2
                || currentSuperState == CurrentSuperState.INTAKING_ALGAE_L3)) {
          if (AutoAlignPoseGenerator.getReefFaceIndex() % 2 == 0) {
            currentSuperState = CurrentSuperState.INTAKING_ALGAE_L3;
            break;
          } else {
            currentSuperState = CurrentSuperState.INTAKING_ALGAE_L2;
            break;
          }
        } else {
          currentSuperState = CurrentSuperState.AUTO_ALIGN_MIDDLE_ALGAE;
          break;
        }
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
      case AUTO_ALIGN_LEFT_BRANCH:
        currentSuperState = CurrentSuperState.AUTO_ALIGN_LEFT_BRANCH;
        break;
      case AUTO_ALIGN_RIGHT_BRANCH:
        currentSuperState = CurrentSuperState.AUTO_ALIGN_RIGHT_BRANCH;
        break;
      case AUTO_ALIGN_MIDDLE_ALGAE:
        currentSuperState = CurrentSuperState.AUTO_ALIGN_MIDDLE_ALGAE;
        break;
      case AUTO_SCORE_L1_LEFT:
        if (!rollers.isCoralDetected()) {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
          wantedSuperState = WantedSuperState.INTAKING_CORAL_STATION;
          break;
        } else if (drivetrain.isAtDriveToPointSetpoints()
            && safetyChecker.isReadyToScore()
            && (currentSuperState == CurrentSuperState.POSITION_CORAL_L1
                || currentSuperState == CurrentSuperState.SCORING_CORAL)) {
          currentSuperState = CurrentSuperState.SCORING_CORAL;
          break;
        } else if (drivetrain.isWithinCoralRaiseDistance()
            && (currentSuperState == CurrentSuperState.AUTO_ALIGN_LEFT_BRANCH
                || currentSuperState == CurrentSuperState.POSITION_CORAL_L1)) {
          currentSuperState = CurrentSuperState.POSITION_CORAL_L1;
          break;
        } else {
          currentSuperState = CurrentSuperState.AUTO_ALIGN_LEFT_BRANCH;
          break;
        }
      case AUTO_SCORE_L1_RIGHT:
        if (!rollers.isCoralDetected()) {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
          wantedSuperState = WantedSuperState.INTAKING_CORAL_STATION;
          break;
        } else if (drivetrain.isAtDriveToPointSetpoints()
            && safetyChecker.isReadyToScore()
            && (currentSuperState == CurrentSuperState.POSITION_CORAL_L1
                || currentSuperState == CurrentSuperState.SCORING_CORAL)) {
          currentSuperState = CurrentSuperState.SCORING_CORAL;
          break;
        } else if (drivetrain.isWithinCoralRaiseDistance()
            && (currentSuperState == CurrentSuperState.AUTO_ALIGN_RIGHT_BRANCH
                || currentSuperState == CurrentSuperState.POSITION_CORAL_L1)) {
          currentSuperState = CurrentSuperState.POSITION_CORAL_L1;
          break;
        } else {
          currentSuperState = CurrentSuperState.AUTO_ALIGN_RIGHT_BRANCH;
          break;
        }
      case AUTO_SCORE_L2_LEFT:
        if (!rollers.isCoralDetected()) {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
          wantedSuperState = WantedSuperState.INTAKING_CORAL_STATION;
          break;
        } else if (drivetrain.isAtDriveToPointSetpoints()
            && safetyChecker.isReadyToScore()
            && (currentSuperState == CurrentSuperState.POSITION_CORAL_L2
                || currentSuperState == CurrentSuperState.SCORING_CORAL)) {
          currentSuperState = CurrentSuperState.SCORING_CORAL;
          break;
        } else if (drivetrain.isWithinCoralRaiseDistance()
            && (currentSuperState == CurrentSuperState.AUTO_ALIGN_LEFT_BRANCH
                || currentSuperState == CurrentSuperState.POSITION_CORAL_L2)) {
          currentSuperState = CurrentSuperState.POSITION_CORAL_L2;
          break;
        } else {
          currentSuperState = CurrentSuperState.AUTO_ALIGN_LEFT_BRANCH;
          break;
        }
      case AUTO_SCORE_L2_RIGHT:
        if (!rollers.isCoralDetected()) {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
          wantedSuperState = WantedSuperState.INTAKING_CORAL_STATION;
          break;
        } else if (drivetrain.isAtDriveToPointSetpoints()
            && safetyChecker.isReadyToScore()
            && (currentSuperState == CurrentSuperState.POSITION_CORAL_L2
                || currentSuperState == CurrentSuperState.SCORING_CORAL)) {
          currentSuperState = CurrentSuperState.SCORING_CORAL;
          break;
        } else if (drivetrain.isWithinCoralRaiseDistance()
            && (currentSuperState == CurrentSuperState.AUTO_ALIGN_RIGHT_BRANCH
                || currentSuperState == CurrentSuperState.POSITION_CORAL_L2)) {
          currentSuperState = CurrentSuperState.POSITION_CORAL_L2;
          break;
        } else {
          currentSuperState = CurrentSuperState.AUTO_ALIGN_RIGHT_BRANCH;
          break;
        }
      case AUTO_SCORE_L3_LEFT:
        if (!rollers.isCoralDetected()) {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
          wantedSuperState = WantedSuperState.INTAKING_CORAL_STATION;
          break;
        } else if (drivetrain.isAtDriveToPointSetpoints()
            && safetyChecker.isReadyToScore()
            && (currentSuperState == CurrentSuperState.POSITION_CORAL_L3
                || currentSuperState == CurrentSuperState.SCORING_CORAL)) {
          currentSuperState = CurrentSuperState.SCORING_CORAL;
          break;
        } else if (drivetrain.isWithinCoralRaiseDistance()
            && (currentSuperState == CurrentSuperState.AUTO_ALIGN_LEFT_BRANCH
                || currentSuperState == CurrentSuperState.POSITION_CORAL_L3)) {
          currentSuperState = CurrentSuperState.POSITION_CORAL_L3;
          break;
        } else {
          currentSuperState = CurrentSuperState.AUTO_ALIGN_LEFT_BRANCH;
          break;
        }
      case AUTO_SCORE_L3_RIGHT:
        if (!rollers.isCoralDetected()) {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
          wantedSuperState = WantedSuperState.INTAKING_CORAL_STATION;
          break;
        } else if (drivetrain.isAtDriveToPointSetpoints()
            && safetyChecker.isReadyToScore()
            && (currentSuperState == CurrentSuperState.POSITION_CORAL_L3
                || currentSuperState == CurrentSuperState.SCORING_CORAL)) {
          currentSuperState = CurrentSuperState.SCORING_CORAL;
          break;
        } else if (drivetrain.isWithinCoralRaiseDistance()
            && (currentSuperState == CurrentSuperState.AUTO_ALIGN_RIGHT_BRANCH
                || currentSuperState == CurrentSuperState.POSITION_CORAL_L3)) {
          currentSuperState = CurrentSuperState.POSITION_CORAL_L3;
          break;
        } else {
          currentSuperState = CurrentSuperState.AUTO_ALIGN_RIGHT_BRANCH;
          break;
        }
      case AUTO_SCORE_L4_LEFT:
        if (!rollers.isCoralDetected()) {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
          wantedSuperState = WantedSuperState.INTAKING_CORAL_STATION;
          break;
        } else if (drivetrain.isAtDriveToPointSetpoints()
            && safetyChecker.isReadyToScore()
            && (currentSuperState == CurrentSuperState.POSITION_CORAL_L4
                || currentSuperState == CurrentSuperState.SCORING_CORAL)) {
          currentSuperState = CurrentSuperState.SCORING_CORAL;
          break;
        } else if (drivetrain.isWithinCoralRaiseDistance()
            && (currentSuperState == CurrentSuperState.AUTO_ALIGN_LEFT_BRANCH
                || currentSuperState == CurrentSuperState.POSITION_CORAL_L4)) {
          currentSuperState = CurrentSuperState.POSITION_CORAL_L4;
          break;
        } else {
          currentSuperState = CurrentSuperState.AUTO_ALIGN_LEFT_BRANCH;
          break;
        }
      case AUTO_SCORE_L4_RIGHT:
        if (!rollers.isCoralDetected()) {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
          wantedSuperState = WantedSuperState.INTAKING_CORAL_STATION;
          break;
        } else if (drivetrain.isAtDriveToPointSetpoints()
            && safetyChecker.isReadyToScore()
            && (currentSuperState == CurrentSuperState.POSITION_CORAL_L4
                || currentSuperState == CurrentSuperState.SCORING_CORAL)) {
          currentSuperState = CurrentSuperState.SCORING_CORAL;
          break;
        } else if (drivetrain.isWithinCoralRaiseDistance()
            && (currentSuperState == CurrentSuperState.AUTO_ALIGN_RIGHT_BRANCH
                || currentSuperState == CurrentSuperState.POSITION_CORAL_L4)) {
          currentSuperState = CurrentSuperState.POSITION_CORAL_L4;
          break;
        } else {
          currentSuperState = CurrentSuperState.AUTO_ALIGN_RIGHT_BRANCH;
          break;
        }
      case POSITION_ALGAE_PROCESSOR:
        currentSuperState = CurrentSuperState.POSITION_ALGAE_PROCESSOR;
        break;
      case POSITION_ALGAE_BARGE:
        currentSuperState = CurrentSuperState.POSITION_ALGAE_BARGE;
        break;
        // case POSITION_CLIMB_PREPARED:
        //   if (climb.isCageDetected()) {
        //     currentSuperState = CurrentSuperState.CLIMBING;
        //     wantedSuperState = WantedSuperState.CLIMBING;
        //   } else {
        //     currentSuperState = CurrentSuperState.POSITION_CLIMB_PREPARED;
        //   }
        //   break;
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
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
          wantedSuperState = WantedSuperState.INTAKING_CORAL_STATION;
        } else {
          currentSuperState = CurrentSuperState.SCORING_ALGAE;
        }
        break;
        // case CLIMBING:
        //   if (!climb.isCageDetected()) {
        //     currentSuperState = CurrentSuperState.POSITION_CLIMB_PREPARED;
        //     wantedSuperState = WantedSuperState.POSITION_CLIMB_PREPARED;
        //   } else {
        //     currentSuperState = CurrentSuperState.CLIMBING;
        //   }
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
      case ENSURING_CORAL:
        ensureCoral();
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
      case AUTO_ALIGN_LEFT_BRANCH:
        autoAlignToBranch(true);
        break;
      case AUTO_ALIGN_RIGHT_BRANCH:
        autoAlignToBranch(false);
        break;
      case AUTO_ALIGN_MIDDLE_ALGAE:
        autoAlignToAlgaeReefFace(false);
        break;
      case AUTO_ALIGN_MIDDLE_BACK_AND_POSITION_ALGAE_PROCESSOR:
        autoAlignToAlgaeReefFace(true);
        positionToAlgaeProcessor();
        break;
      case POSITION_ALGAE_PROCESSOR:
        positionToAlgaeProcessor();
        break;
      case POSITION_ALGAE_BARGE:
        positionToAlgaeBarge();
        break;
      case POSITION_CLIMB_PREPARED:
        positionToClimbPrepared();
        break;
      case SCORING_CORAL:
        scoreCoral();
        break;
      case SCORING_ALGAE:
        scoreAlgae();
        break;
      case CLIMBING:
        climbing();
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
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void ensureCoral() {
    elevator.setWantedState(Elevator.WantedState.INTAKING_CORAL_STATION);
    rollers.setWantedState(Rollers.WantedState.ENSURING_CORAL);
    wrist.setWantedState(Wrist.WantedState.INTAKING_CORAL_STATION);
    leds.setCurrentState(LEDs.CurrentState.ENSURING_CORAL);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void intakeAlgaeGround() {
    elevator.setWantedState(Elevator.WantedState.INTAKING_ALGAE_GROUND);
    rollers.setWantedState(Rollers.WantedState.INTAKING_ALGAE);
    wrist.setWantedState(Wrist.WantedState.INTAKING_ALGAE_GROUND);
    leds.setCurrentState(LEDs.CurrentState.INTAKING_ALGAE_GROUND);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void intakeAlgaeL2() {
    elevator.setWantedState(Elevator.WantedState.INTAKING_ALGAE_L2);
    rollers.setWantedState(Rollers.WantedState.INTAKING_ALGAE);
    wrist.setWantedState(Wrist.WantedState.INTAKING_ALGAE_REEF);
    leds.setCurrentState(LEDs.CurrentState.INTAKING_ALGAE_L2);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void intakeAlgaeL3() {
    elevator.setWantedState(Elevator.WantedState.INTAKING_ALGAE_L3);
    rollers.setWantedState(Rollers.WantedState.INTAKING_ALGAE);
    wrist.setWantedState(Wrist.WantedState.INTAKING_ALGAE_REEF);
    leds.setCurrentState(LEDs.CurrentState.INTAKING_ALGAE_L3);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void prepare() {
    elevator.setWantedState(Elevator.WantedState.POSITION_PREPARED);
    rollers.setWantedState(Rollers.WantedState.STOPPED);
    wrist.setWantedState(Wrist.WantedState.POSITION_PREPARED);
    leds.setCurrentState(LEDs.CurrentState.POSITION_PREPARED);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void positionToCoralL1() {
    elevator.setWantedState(Elevator.WantedState.POSITION_CORAL_L1);
    rollers.setWantedState(Rollers.WantedState.STOPPED);
    wrist.setWantedState(Wrist.WantedState.SCORING_CORAL);
    leds.setCurrentState(LEDs.CurrentState.POSITION_CORAL_L1);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void positionToCoralL2() {
    elevator.setWantedState(Elevator.WantedState.POSITION_CORAL_L2);
    rollers.setWantedState(Rollers.WantedState.STOPPED);
    wrist.setWantedState(Wrist.WantedState.SCORING_CORAL);
    leds.setCurrentState(LEDs.CurrentState.POSITION_CORAL_L2);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void positionToCoralL3() {
    elevator.setWantedState(Elevator.WantedState.POSITION_CORAL_L3);
    rollers.setWantedState(Rollers.WantedState.STOPPED);
    wrist.setWantedState(Wrist.WantedState.SCORING_CORAL);
    leds.setCurrentState(LEDs.CurrentState.POSITION_CORAL_L3);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void positionToCoralL4() {
    elevator.setWantedState(Elevator.WantedState.POSITION_CORAL_L4);
    rollers.setWantedState(Rollers.WantedState.STOPPED);
    wrist.setWantedState(Wrist.WantedState.SCORING_CORAL);
    leds.setCurrentState(LEDs.CurrentState.POSITION_CORAL_L4);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void positionToAlgaeProcessor() {
    elevator.setWantedState(Elevator.WantedState.POSITION_ALGAE_PROCESSOR);
    if (rollers.isAlgaeDetected()) {
      rollers.setWantedState(Rollers.WantedState.HOLD_ALGAE);
    } else {
      rollers.setWantedState(Rollers.WantedState.STOPPED);
    }
    wrist.setWantedState(Wrist.WantedState.SCORING_ALGAE);
    leds.setCurrentState(LEDs.CurrentState.POSITION_ALGAE_PROCESSOR);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void positionToAlgaeBarge() {
    elevator.setWantedState(Elevator.WantedState.POSITION_ALGAE_BARGE);
    if (rollers.isAlgaeDetected()) {
      rollers.setWantedState(Rollers.WantedState.HOLD_ALGAE);
    } else {
      rollers.setWantedState(Rollers.WantedState.STOPPED);
    }
    wrist.setWantedState(Wrist.WantedState.SCORING_ALGAE);
    leds.setCurrentState(LEDs.CurrentState.POSITION_ALGAE_BARGE);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void positionToClimbPrepared() {
    elevator.setWantedState(Elevator.WantedState.INTAKING_CORAL_STATION);
    rollers.setWantedState(Rollers.WantedState.STOPPED);
    wrist.setWantedState(Wrist.WantedState.INTAKING_CORAL_STATION);
    // climb.setWantedState(Climb.WantedState.CLIMB_PREPARED);
    leds.setCurrentState(LEDs.CurrentState.POSITION_CLIMB_PREPARED);
  }

  /**
   * Automatically drives to a branch
   *
   * @param useLeftBranch true if the target branch is the left branch, false if it is the right
   *     branch
   */
  private void autoAlignToBranch(boolean useLeftBranch) {
    drivetrain.setTargetPoseForDriveToPoint(
        AutoAlignPoseGenerator.getNearestBranchPosition(drivetrain.getPose(), useLeftBranch));
    drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.DRIVE_TO_POINT);
  }

  /**
   * Automatically drives to a reef face
   *
   * @param shiftBackFromReefFace true if the target pose should be shifted back from the reef face,
   *     false if the target pose should be right up to the reef face
   */
  private void autoAlignToAlgaeReefFace(boolean shiftBackFromReefFace) {
    drivetrain.setTargetPoseForDriveToPoint(
        AutoAlignPoseGenerator.getNearestAlgaeReefFacePosition(
            drivetrain.getPose(), shiftBackFromReefFace));
    drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.DRIVE_TO_POINT);
  }

  private void scoreCoral() {
    rollers.setWantedState(Rollers.WantedState.SCORING_CORAL);
    leds.setCurrentState(LEDs.CurrentState.SCORING_CORAL);
  }

  private void scoreAlgae() {
    rollers.setWantedState(Rollers.WantedState.SCORING_ALGAE);
    leds.setCurrentState(LEDs.CurrentState.SCORING_ALGAE);
  }

  private void climbing() {
    // climb.setWantedState(Climb.WantedState.CLIMBING);
    leds.setCurrentState(LEDs.CurrentState.CLIMBING);
  }

  private void stop() {
    elevator.setWantedState(Elevator.WantedState.STOPPED);
    rollers.setWantedState(Rollers.WantedState.STOPPED);
    wrist.setWantedState(Wrist.WantedState.STOPPED);
    leds.setCurrentState(LEDs.CurrentState.STOPPED);
    // climb.setWantedState(Climb.WantedState.STOPPED);
  }

  public WantedSuperState returnAutoAlgaeIntakeState() {
    if (automationLevel == AutomationLevel.AUTO_ACTION) {
      return WantedSuperState.AUTO_INTAKE_ALGAE;
    } else {
      return WantedSuperState.AUTO_ALIGN_MIDDLE_ALGAE;
    }
  }

  public void setCoralQueuedState() {
    if (automationLevel == AutomationLevel.AUTO_ACTION) {
      switch (queuedSuperState) {
        case POSITION_CORAL_L1:
          queuedSuperState = WantedSuperState.AUTO_SCORE_L1;
          break;
        case POSITION_CORAL_L2:
          queuedSuperState = WantedSuperState.AUTO_SCORE_L2;
          break;
        case POSITION_CORAL_L3:
          queuedSuperState = WantedSuperState.AUTO_SCORE_L3;
          break;
        case POSITION_CORAL_L4:
          queuedSuperState = WantedSuperState.AUTO_SCORE_L4;
          break;
        default:
          break;
      }
    }
  }

  public WantedSuperState returnAutoCoralScoreState(boolean alignLeft) {
    setCoralQueuedState();
    if (alignLeft) {
      switch (queuedSuperState) {
        case AUTO_SCORE_L1:
          return WantedSuperState.AUTO_SCORE_L1_LEFT;
        case AUTO_SCORE_L2:
          return WantedSuperState.AUTO_SCORE_L2_LEFT;
        case AUTO_SCORE_L3:
          return WantedSuperState.AUTO_SCORE_L3_LEFT;
        case AUTO_SCORE_L4:
          return WantedSuperState.AUTO_SCORE_L4_LEFT;
        default:
          return WantedSuperState.AUTO_ALIGN_LEFT_BRANCH;
      }
    } else {
      switch (queuedSuperState) {
        case AUTO_SCORE_L1:
          return WantedSuperState.AUTO_SCORE_L1_RIGHT;
        case AUTO_SCORE_L2:
          return WantedSuperState.AUTO_SCORE_L2_RIGHT;
        case AUTO_SCORE_L3:
          return WantedSuperState.AUTO_SCORE_L3_RIGHT;
        case AUTO_SCORE_L4:
          return WantedSuperState.AUTO_SCORE_L4_RIGHT;
        default:
          return WantedSuperState.AUTO_ALIGN_RIGHT_BRANCH;
      }
    }
  }

  public Command setTeleopDriveStateCommand() {
    return this.runOnce(() -> setTeleopDriveState());
  }

  public void setTeleopDriveState() {
    drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.TELEOP_DRIVE);
    switch (currentSuperState) {
      case AUTO_ALIGN_LEFT_BRANCH:
        wantedSuperState = WantedSuperState.POSITION_PREPARED;
        break;
      case AUTO_ALIGN_RIGHT_BRANCH:
        wantedSuperState = WantedSuperState.POSITION_PREPARED;
        break;
      case AUTO_ALIGN_MIDDLE_ALGAE:
        wantedSuperState = WantedSuperState.POSITION_PREPARED;
        break;
      case AUTO_ALIGN_MIDDLE_BACK_AND_POSITION_ALGAE_PROCESSOR:
        wantedSuperState = WantedSuperState.POSITION_ALGAE_PROCESSOR;
        break;
      case POSITION_CORAL_L1:
        wantedSuperState = WantedSuperState.POSITION_CORAL_L1;
        break;
      case POSITION_CORAL_L2:
        wantedSuperState = WantedSuperState.POSITION_CORAL_L2;
        break;
      case POSITION_CORAL_L3:
        wantedSuperState = WantedSuperState.POSITION_CORAL_L3;
        break;
      case POSITION_CORAL_L4:
        wantedSuperState = WantedSuperState.POSITION_CORAL_L4;
        break;
      case INTAKING_ALGAE_L2:
        wantedSuperState = WantedSuperState.INTAKING_ALGAE_L2;
        break;
      case INTAKING_ALGAE_L3:
        wantedSuperState = WantedSuperState.INTAKING_ALGAE_L3;
        break;
      case SCORING_CORAL:
        wantedSuperState = WantedSuperState.SCORING_CORAL;
        break;
      default:
        break;
    }
  }

  public Command setQueuedSuperStateCommand(WantedSuperState nextState) {
    return Commands.parallel(
        this.runOnce(() -> setQueuedSuperState(nextState)), rumbleControllers().withTimeout(0.2));
  }

  private void setQueuedSuperState(WantedSuperState state) {
    queuedSuperState = state;
  }

  public Command updateWantedSuperStateCommand() {
    return Commands.parallel(
        this.runOnce(() -> updateWantedSuperState()), rumbleControllers().withTimeout(0.3));
  }

  private void updateWantedSuperState() {
    wantedSuperState = queuedSuperState;
  }

  public Command setWantedSuperStateCommand(WantedSuperState wantedState) {
    return this.runOnce(() -> setWantedSuperState(wantedState));
  }

  private void setWantedSuperState(WantedSuperState state) {
    wantedSuperState = state;
  }

  public Command setAutomationLevelCommand(AutomationLevel level) {
    return this.runOnce(() -> setAutomationLevel(level));
  }

  public void setAutomationLevel(AutomationLevel level) {
    automationLevel = level;
  }

  public Command zeroGyroCommand() {
    return this.runOnce(() -> drivetrain.zeroGyro());
  }

  public Command rumbleControllers() {
    return new StartEndCommand(
            () -> driver.getHID().setRumble(RumbleType.kBothRumble, 1),
            () -> driver.getHID().setRumble(RumbleType.kBothRumble, 0))
        .alongWith(
            new StartEndCommand(
                () -> operator.getHID().setRumble(RumbleType.kBothRumble, 1),
                () -> operator.getHID().setRumble(RumbleType.kBothRumble, 0)))
        .withTimeout(0.5);
  }

  /* USED FOR AUTOS ONLY */
  public boolean isWithinCoralRaiseDistance() {
    return drivetrain.isWithinCoralRaiseDistance();
  }

  public boolean isWithinAlgaeRaiseDistance() {
    return drivetrain.isWithinAlgaeRaiseDistance();
  }

  public boolean hasCoral() {
    return rollers.isCoralDetected();
  }
}
