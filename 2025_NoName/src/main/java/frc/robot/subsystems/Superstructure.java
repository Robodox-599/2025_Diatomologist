package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.SafetyChecker;
import frc.robot.commands.AutoAlignPoseGenerator;
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
  // private final Climb climb;
  // private final Vision vision;
  private final SafetyChecker safetyChecker;
  private final CommandXboxController driver;
  private final CommandXboxController operator;

  private CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;
  private WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
  private WantedSuperState queuedSuperState = WantedSuperState.STOPPED;

  public enum CurrentSuperState {
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
    AUTO_SCORE_L1_LEFT,
    AUTO_SCORE_L1_RIGHT,
    AUTO_SCORE_L2_LEFT,
    AUTO_SCORE_L2_RIGHT,
    AUTO_SCORE_L3_LEFT,
    AUTO_SCORE_L3_RIGHT,
    AUTO_SCORE_L4_LEFT,
    AUTO_SCORE_L4_RIGHT,
    POSITION_ALGAE_PROCESSOR,
    POSITION_ALGAE_BARGE,
    POSITION_CLIMB_PREPARED,
    AUTO_ALIGN_LEFT_BRANCH,
    AUTO_ALIGN_RIGHT_BRANCH,
    AUTO_ALIGN_ALGAE,
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
    AUTO_SCORE_LEFT,
    AUTO_SCORE_RIGHT,
    POSITION_ALGAE_PROCESSOR,
    POSITION_ALGAE_BARGE,
    POSITION_CLIMB_PREPARED,
    SCORING_CORAL,
    SCORING_ALGAE,
    CLIMBING,
    STOPPED,
    NO_STATE,
  }

  public Superstructure(
      CommandSwerveDrivetrain drivetrain,
      Elevator elevator,
      Wrist wrist,
      Rollers rollers,
      LEDs LEDs,
      // Climb climb,
      // Vision vision,
      SafetyChecker safetyChecker,
      CommandXboxController driver,
      CommandXboxController operator) {
    this.drivetrain = drivetrain;
    this.elevator = elevator;
    this.wrist = wrist;
    this.rollers = rollers;
    this.leds = LEDs;
    // this.climb = climb;
    // this.vision = vision;
    this.safetyChecker = safetyChecker;
    this.driver = driver;
    this.operator = operator;
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      currentSuperState = CurrentSuperState.STOPPED;
      wantedSuperState = WantedSuperState.STOPPED;
      queuedSuperState = WantedSuperState.STOPPED;
    }
    drivetrain.updateInputs();
    elevator.updateInputs();
    rollers.updateInputs();
    wrist.updateInputs();
    leds.updateInputs();
    // climb.updateInputs();
    // vision.updateInputs();
    currentSuperState = handleStateTransitions();
    applyStates();

    DogLog.log("Superstructure/QueuedSuperState", queuedSuperState);
    DogLog.log("Superstructure/WantedSuperState", wantedSuperState);
    DogLog.log("Superstructure/CurrentSuperState", currentSuperState);
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
        currentSuperState = CurrentSuperState.AUTO_INTAKE_ALGAE;
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
      case AUTO_SCORE_LEFT:
        switch (queuedSuperState) {
          case POSITION_CORAL_L1:
            currentSuperState = CurrentSuperState.AUTO_SCORE_L1_LEFT;
            break;
          case POSITION_CORAL_L2:
            currentSuperState = CurrentSuperState.AUTO_SCORE_L2_LEFT;
            break;
          case POSITION_CORAL_L3:
            currentSuperState = CurrentSuperState.AUTO_SCORE_L3_LEFT;
            break;
          case POSITION_CORAL_L4:
            currentSuperState = CurrentSuperState.AUTO_SCORE_L4_LEFT;
            break;
          default:
            currentSuperState = CurrentSuperState.AUTO_ALIGN_LEFT_BRANCH;
            break;
        }
        break;
      case AUTO_SCORE_RIGHT:
        switch (queuedSuperState) {
          case POSITION_CORAL_L1:
            currentSuperState = CurrentSuperState.AUTO_SCORE_L1_RIGHT;
            break;
          case POSITION_CORAL_L2:
            currentSuperState = CurrentSuperState.AUTO_SCORE_L2_RIGHT;
            break;
          case POSITION_CORAL_L3:
            currentSuperState = CurrentSuperState.AUTO_SCORE_L3_RIGHT;
            break;
          case POSITION_CORAL_L4:
            currentSuperState = CurrentSuperState.AUTO_SCORE_L4_RIGHT;
            break;
          default:
            currentSuperState = CurrentSuperState.AUTO_ALIGN_RIGHT_BRANCH;
            break;
        }
        break;
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
      case AUTO_INTAKE_ALGAE:
        autoIntakeAlgae();
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
      case AUTO_SCORE_L1_LEFT:
        autoScoreL1(true);
        break;
      case AUTO_SCORE_L1_RIGHT:
        autoScoreL1(false);
        break;
      case AUTO_SCORE_L2_LEFT:
        autoScoreL2(true);
        break;
      case AUTO_SCORE_L2_RIGHT:
        autoScoreL2(false);
        break;
      case AUTO_SCORE_L3_LEFT:
        autoScoreL3(true);
        break;
      case AUTO_SCORE_L3_RIGHT:
        autoScoreL3(false);
        break;
      case AUTO_SCORE_L4_LEFT:
        autoScoreL4(true);
        break;
      case AUTO_SCORE_L4_RIGHT:
        autoScoreL4(false);
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
      case AUTO_ALIGN_LEFT_BRANCH:
        autoAlignToBranch(true);
        break;
      case AUTO_ALIGN_RIGHT_BRANCH:
        autoAlignToBranch(false);
        break;
      case AUTO_ALIGN_ALGAE:
        autoAlignToAlgaeReefFace();
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

  private void autoIntakeAlgae() {
    if (rollers.isAlgaeDetected()) {
      drivetrain.setTargetPoseForDriveToPoint(
          AutoAlignPoseGenerator.getNearestAlgaeReefFacePosition(drivetrain.getPose(), true));
      positionToAlgaeProcessor();
      if (drivetrain.isAtDriveToPointSetpoints()) {
        setWantedSuperState(WantedSuperState.POSITION_ALGAE_PROCESSOR);
      }
    }
    drivetrain.setTargetPoseForDriveToPoint(
        AutoAlignPoseGenerator.getNearestAlgaeReefFacePosition(drivetrain.getPose(), false));
    drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.DRIVE_TO_POINT);
    if (drivetrain.isWithinAlgaeRaiseDistance()) {
      if (AutoAlignPoseGenerator.getReefFaceIndex() % 2 == 0) {
        intakeAlgaeL3();
      } else {
        intakeAlgaeL2();
      }
    }
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

  /**
   * Automatically drives to a branch and scores L1
   *
   * @param useLeftBranch true if the target branch is the left branch, false if it is the right
   *     branch
   */
  private void autoScoreL1(boolean useLeftBranch) {
    if (!rollers.isCoralDetected()) {
      wantedSuperState = WantedSuperState.INTAKING_CORAL_STATION;
    }
    drivetrain.setTargetPoseForDriveToPoint(
        AutoAlignPoseGenerator.getNearestBranchPosition(drivetrain.getPose(), useLeftBranch));
    drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.DRIVE_TO_POINT);
    if (drivetrain.isWithinCoralRaiseDistance()) {
      positionToCoralL1();
      if (drivetrain.isAtDriveToPointSetpoints() && safetyChecker.isReadyToScore()) {
        scoreCoral();
      }
    }
  }

  /**
   * Automatically drives to a branch and scores L2
   *
   * @param useLeftBranch true if the target branch is the left branch, false if it is the right
   *     branch
   */
  private void autoScoreL2(boolean useLeftBranch) {
    if (!rollers.isCoralDetected()) {
      wantedSuperState = WantedSuperState.INTAKING_CORAL_STATION;
    }
    drivetrain.setTargetPoseForDriveToPoint(
        AutoAlignPoseGenerator.getNearestBranchPosition(drivetrain.getPose(), useLeftBranch));
    drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.DRIVE_TO_POINT);
    if (drivetrain.isWithinCoralRaiseDistance()) {
      positionToCoralL2();
      if (drivetrain.isAtDriveToPointSetpoints() && safetyChecker.isReadyToScore()) {
        scoreCoral();
      }
    }
  }

  /**
   * Automatically drives to a branch and scores L3
   *
   * @param useLeftBranch true if the target branch is the left branch, false if it is the right
   *     branch
   */
  private void autoScoreL3(boolean useLeftBranch) {
    if (!rollers.isCoralDetected()) {
      wantedSuperState = WantedSuperState.INTAKING_CORAL_STATION;
    }
    drivetrain.setTargetPoseForDriveToPoint(
        AutoAlignPoseGenerator.getNearestBranchPosition(drivetrain.getPose(), useLeftBranch));
    drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.DRIVE_TO_POINT);
    if (drivetrain.isWithinCoralRaiseDistance()) {
      positionToCoralL3();
      if (drivetrain.isAtDriveToPointSetpoints() && safetyChecker.isReadyToScore()) {
        scoreCoral();
      }
    }
  }

  /**
   * Automatically drives to a branch and scores L4
   *
   * @param useLeftBranch true if the target branch is the left branch, false if it is the right
   *     branch
   */
  private void autoScoreL4(boolean useLeftBranch) {
    if (!rollers.isCoralDetected()) {
      wantedSuperState = WantedSuperState.INTAKING_CORAL_STATION;
    }
    drivetrain.setTargetPoseForDriveToPoint(
        AutoAlignPoseGenerator.getNearestBranchPosition(drivetrain.getPose(), useLeftBranch));
    drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.DRIVE_TO_POINT);
    if (drivetrain.isWithinCoralRaiseDistance()) {
      positionToCoralL4();
      if (drivetrain.isAtDriveToPointSetpoints() && safetyChecker.isReadyToScore()) {
        scoreCoral();
      }
    }
  }

  private void positionToAlgaeProcessor() {
    elevator.setWantedState(Elevator.WantedState.POSITION_ALGAE_PROCESSOR);
    // rollers.setWantedState(Rollers.WantedState.HOLD_ALGAE);
    wrist.setWantedState(Wrist.WantedState.SCORING_ALGAE);
    leds.setCurrentState(LEDs.CurrentState.POSITION_ALGAE_PROCESSOR);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void positionToAlgaeBarge() {
    elevator.setWantedState(Elevator.WantedState.POSITION_ALGAE_BARGE);
    // rollers.setWantedState(Rollers.WantedState.HOLD_ALGAE);
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

  /** Automatically drives to a reef face */
  private void autoAlignToAlgaeReefFace() {
    drivetrain.setTargetPoseForDriveToPoint(
        AutoAlignPoseGenerator.getNearestAlgaeReefFacePosition(drivetrain.getPose(), false));
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

  public Command setQueuedSuperStateCommand(WantedSuperState nextState) {
    return Commands.parallel(
        this.runOnce(() -> setQueuedSuperState(nextState)), rumbleControllers().withTimeout(0.2));
  }

  private void setQueuedSuperState(WantedSuperState state) {
    queuedSuperState = state;
  }

  public Command updateWantedSuperStateCommand() {
    return Commands.either(
        Commands.sequence(
            rumbleControllers().withTimeout(0.1),
            new WaitCommand(0.2),
            rumbleControllers().withTimeout(0.1)),
        Commands.parallel(
            this.runOnce(() -> updateWantedSuperState()), rumbleControllers().withTimeout(0.3)),
        (() -> queuedSuperState == WantedSuperState.NO_STATE));
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

  public Command zeroGyroCommand() {
    return this.runOnce(() -> drivetrain.zeroGyro());
  }

  public void setTeleopDriveStateAndPrepare() {
    drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.TELEOP_DRIVE);
    wantedSuperState = WantedSuperState.POSITION_PREPARED;
  }

  public Command setTeleopDriveStateAndPrepareCommand() {
    return this.runOnce(() -> setTeleopDriveStateAndPrepare());
  }

  public Command setTeleopDriveStateCommand() {
    return this.runOnce(
        () -> drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.TELEOP_DRIVE));
  }

  public boolean isWithinCoralRaiseDistance() {
    return drivetrain.isWithinCoralRaiseDistance();
  }

  public boolean isWithinAlgaeRaiseDistance() {
    return drivetrain.isWithinAlgaeRaiseDistance();
  }

  public boolean hasCoral() {
    return rollers.isCoralDetected();
  }

  public Command rumbleDriverController() {
    return new StartEndCommand(
            () -> driver.getHID().setRumble(RumbleType.kBothRumble, 1),
            () -> driver.getHID().setRumble(RumbleType.kBothRumble, 0))
        .withTimeout(0.5);
  }

  public Command rumbleOperatorController() {
    return new StartEndCommand(
            () -> operator.getHID().setRumble(RumbleType.kBothRumble, 1),
            () -> operator.getHID().setRumble(RumbleType.kBothRumble, 0))
        .withTimeout(0.5);
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
}
