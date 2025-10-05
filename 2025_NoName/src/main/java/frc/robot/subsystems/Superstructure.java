package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.endefector.endefectorrollers.Rollers;
import frc.robot.subsystems.endefector.endefectorwrist.Wrist;
import frc.robot.subsystems.endefector.endefectorwrist.WristConstants.WristStates;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.vision4.Vision4;
import frc.robot.util.AutoAlignPoseGenerator;
import frc.robot.util.Tracer;

public class Superstructure extends SubsystemBase {
  private final CommandSwerveDrivetrain drivetrain;
  private final Elevator elevator;
  private final Wrist wrist;
  private final Rollers rollers;
  private final LEDs leds;
  private final Vision4 vision;
  private final CommandXboxController driver;
  private final CommandXboxController operator;

  private CurrentSuperState previousSuperState = CurrentSuperState.STOPPED;
  private CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;
  private WantedSuperState wantedSuperState = WantedSuperState.STOPPED;

  public enum CurrentSuperState {
    INTAKING_CORAL_STATION,
    INTAKING_ALGAE_GROUND,
    INTAKING_ALGAE_LOLLIPOP,
    POSITION_ALGAE_L2,
    INTAKING_ALGAE_L2,
    POSITION_ALGAE_L3,
    INTAKING_ALGAE_L3,
    POSITION_PREPARED,
    POSITION_PREPARED_AUTO,
    POSITION_CORAL_L1,
    POSITION_CORAL_L2,
    POSITION_CORAL_L3,
    POSITION_CORAL_L4,
    POSITION_CORAL_L4_AUTO,
    AUTO_ALIGN_LEFT_TROUGH,
    AUTO_ALIGN_LEFT_BRANCH_L2,
    AUTO_ALIGN_LEFT_BRANCH_L3,
    AUTO_ALIGN_LEFT_BRANCH_L4,
    AUTO_ALIGN_LEFT_BRANCH_L4_AUTO,
    AUTO_ALIGN_MIDDLE_LEFT_TROUGH,
    AUTO_ALIGN_MIDDLE_RIGHT_TROUGH,
    AUTO_ALIGN_RIGHT_TROUGH,
    AUTO_ALIGN_RIGHT_BRANCH_L2,
    AUTO_ALIGN_RIGHT_BRANCH_L3,
    AUTO_ALIGN_RIGHT_BRANCH_L4,
    AUTO_ALIGN_RIGHT_BRANCH_L4_AUTO,
    AUTO_ALIGN_MIDDLE_ALGAE,
    POSITION_ALGAE_PROCESSOR,
    POSITION_ALGAE_BARGE,
    POSITION_CLIMB_PREPARED,
    SCORING_CORAL_TROUGH,
    SCORING_CORAL_L2_L3,
    SCORING_CORAL_L4,
    SCORING_ALGAE,
    CLIMBING,
    STOPPED,
  }

  public enum WantedSuperState {
    INTAKING_CORAL_STATION,
    INTAKING_ALGAE_GROUND,
    INTAKING_ALGAE_LOLLIPOP,
    POSITION_ALGAE_L2,
    INTAKING_ALGAE_L2,
    POSITION_ALGAE_L3,
    INTAKING_ALGAE_L3,
    AUTO_INTAKE_ALGAE,
    POSITION_PREPARED,
    POSITION_PREPARED_AUTO,
    POSITION_CORAL_L1,
    POSITION_CORAL_L2,
    POSITION_CORAL_L3,
    POSITION_CORAL_L4,
    POSITION_CORAL_L4_AUTO,
    AUTO_ALIGN_LEFT_TROUGH,
    AUTO_ALIGN_LEFT_BRANCH_L2,
    AUTO_ALIGN_LEFT_BRANCH_L3,
    AUTO_ALIGN_LEFT_BRANCH_L4,
    AUTO_ALIGN_MIDDLE_LEFT_TROUGH,
    AUTO_ALIGN_MIDDLE_RIGHT_TROUGH,
    AUTO_ALIGN_RIGHT_TROUGH,
    AUTO_ALIGN_RIGHT_BRANCH_L2,
    AUTO_ALIGN_RIGHT_BRANCH_L3,
    AUTO_ALIGN_RIGHT_BRANCH_L4,
    AUTO_ALIGN_MIDDLE_ALGAE,
    AUTO_SCORE_L1_LEFT,
    AUTO_SCORE_L1_MIDDLE_LEFT,
    AUTO_SCORE_L1_MIDDLE_RIGHT,
    AUTO_SCORE_L1_RIGHT,
    AUTO_SCORE_L2_LEFT,
    AUTO_SCORE_L2_RIGHT,
    AUTO_SCORE_L3_LEFT,
    AUTO_SCORE_L3_RIGHT,
    AUTO_SCORE_L4_LEFT,
    AUTO_SCORE_L4_RIGHT,
    AUTO_SCORE_L4_LEFT_AUTO,
    AUTO_SCORE_L4_RIGHT_AUTO,
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
      Vision4 vision,
      CommandXboxController driver,
      CommandXboxController operator) {
    this.drivetrain = drivetrain;
    this.elevator = elevator;
    this.wrist = wrist;
    this.rollers = rollers;
    this.leds = LEDs;
    this.vision = vision;
    this.driver = driver;
    this.operator = operator;
  }

  @Override
  public void periodic() {
    Tracer.startTrace("SuperstructurePeriodic");
    Tracer.traceFunc(
        "AutoAlignPoseGeneratorUpdate",
        () -> AutoAlignPoseGenerator.updateNearestReefFaceIndex(drivetrain.getPose()));
    Tracer.traceFunc("DrivetrainPeriodic", drivetrain::updateInputs);
    Tracer.traceFunc("VisionPeriodic", vision::updateInputs);
    Tracer.traceFunc("ElevatorPeriodic", elevator::updateInputs);
    Tracer.traceFunc("RollersPeriodic", rollers::updateInputs);
    Tracer.traceFunc("WristPeriodic", wrist::updateInputs);
    Tracer.traceFunc("LedsPeriodic", leds::updateInputs);
    // climb.updateInputs();
    Tracer.traceFunc("HandleStateTransitions", this::handleStateTransitions);
    Tracer.traceFunc("ApplyStates", this::applyStates);

    DogLog.log("Superstructure/CurrentSuperState", currentSuperState);
    DogLog.log("Superstructure/WantedSuperState", wantedSuperState);
    Tracer.endTrace();
  }

  private void handleStateTransitions() {
    previousSuperState = currentSuperState;
    switch (wantedSuperState) {
      case INTAKING_CORAL_STATION:
        if (rollers.isCoralEnsured()) {
          currentSuperState = CurrentSuperState.POSITION_PREPARED;
          wantedSuperState = WantedSuperState.POSITION_PREPARED;
        } else {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
        }
        break;
      case INTAKING_ALGAE_GROUND:
        if (rollers.isAlgaeIntaked()) {
          currentSuperState = CurrentSuperState.POSITION_ALGAE_PROCESSOR;
          wantedSuperState = WantedSuperState.POSITION_ALGAE_PROCESSOR;
        } else {
          currentSuperState = CurrentSuperState.INTAKING_ALGAE_GROUND;
        }
        break;
      case INTAKING_ALGAE_LOLLIPOP:
        if (rollers.isAlgaeIntaked()) {
          currentSuperState = CurrentSuperState.POSITION_ALGAE_PROCESSOR;
          wantedSuperState = WantedSuperState.POSITION_ALGAE_PROCESSOR;
        } else {
          currentSuperState = CurrentSuperState.INTAKING_ALGAE_LOLLIPOP;
        }
        break;
      case POSITION_ALGAE_L2:
        currentSuperState = CurrentSuperState.POSITION_ALGAE_L2;
        break;
      case INTAKING_ALGAE_L2:
        if (rollers.isAlgaeIntaked()) {
          currentSuperState = CurrentSuperState.POSITION_ALGAE_L2;
          wantedSuperState = WantedSuperState.POSITION_ALGAE_PROCESSOR;
        } else {
          currentSuperState = CurrentSuperState.INTAKING_ALGAE_L2;
        }
        break;
      case POSITION_ALGAE_L3:
        currentSuperState = CurrentSuperState.POSITION_ALGAE_L3;
        break;
      case INTAKING_ALGAE_L3:
        if (rollers.isAlgaeIntaked()) {
          currentSuperState = CurrentSuperState.POSITION_ALGAE_L3;
          wantedSuperState = WantedSuperState.POSITION_ALGAE_PROCESSOR;
        } else {
          currentSuperState = CurrentSuperState.INTAKING_ALGAE_L3;
        }
        break;
      case AUTO_INTAKE_ALGAE:
        if (rollers.isAlgaeIntaked()) {
          if (currentSuperState == CurrentSuperState.INTAKING_ALGAE_L2) {
            currentSuperState = CurrentSuperState.POSITION_ALGAE_L2;
          } else if (currentSuperState == CurrentSuperState.INTAKING_ALGAE_L3) {
            currentSuperState = CurrentSuperState.POSITION_ALGAE_L3;
          }
          wantedSuperState = WantedSuperState.POSITION_ALGAE_PROCESSOR;
        } else if (drivetrain.isWithinReefZone()
            && drivetrain.isYErrorWithinTolerance()
            && drivetrain.isDrivingToPointOrAtSetpoints()
            && (currentSuperState == CurrentSuperState.POSITION_ALGAE_L2
                || currentSuperState == CurrentSuperState.POSITION_ALGAE_L3
                || currentSuperState == CurrentSuperState.INTAKING_ALGAE_L2
                || currentSuperState == CurrentSuperState.INTAKING_ALGAE_L3)) {
          if (AutoAlignPoseGenerator.getNearestReefFaceIndex() % 2 == 0) {
            currentSuperState = CurrentSuperState.INTAKING_ALGAE_L3;
          } else {
            currentSuperState = CurrentSuperState.INTAKING_ALGAE_L2;
          }
        } else if (drivetrain.isReadyToRaiseAutoIntakeAlgae()) {
          if (AutoAlignPoseGenerator.getNearestReefFaceIndex() % 2 == 0) {
            currentSuperState = CurrentSuperState.POSITION_ALGAE_L3;
          } else {
            currentSuperState = CurrentSuperState.POSITION_ALGAE_L2;
          }
        } else {
          currentSuperState = CurrentSuperState.AUTO_ALIGN_MIDDLE_ALGAE;
        }
        break;
      case POSITION_PREPARED:
        currentSuperState = CurrentSuperState.POSITION_PREPARED;
        break;
      case POSITION_PREPARED_AUTO:
        currentSuperState = CurrentSuperState.POSITION_PREPARED_AUTO;
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
      case POSITION_CORAL_L4_AUTO:
        currentSuperState = CurrentSuperState.POSITION_CORAL_L4_AUTO;
        break;
      case AUTO_ALIGN_LEFT_TROUGH:
        currentSuperState = CurrentSuperState.AUTO_ALIGN_LEFT_TROUGH;
        break;
      case AUTO_ALIGN_LEFT_BRANCH_L2:
        currentSuperState = CurrentSuperState.AUTO_ALIGN_LEFT_BRANCH_L2;
        break;
      case AUTO_ALIGN_LEFT_BRANCH_L3:
        currentSuperState = CurrentSuperState.AUTO_ALIGN_LEFT_BRANCH_L3;
        break;
      case AUTO_ALIGN_LEFT_BRANCH_L4:
        currentSuperState = CurrentSuperState.AUTO_ALIGN_LEFT_BRANCH_L4;
        break;
      case AUTO_ALIGN_MIDDLE_LEFT_TROUGH:
        currentSuperState = CurrentSuperState.AUTO_ALIGN_MIDDLE_LEFT_TROUGH;
        break;
      case AUTO_ALIGN_MIDDLE_RIGHT_TROUGH:
        currentSuperState = CurrentSuperState.AUTO_ALIGN_MIDDLE_RIGHT_TROUGH;
        break;
      case AUTO_ALIGN_RIGHT_TROUGH:
        currentSuperState = CurrentSuperState.AUTO_ALIGN_RIGHT_TROUGH;
        break;
      case AUTO_ALIGN_RIGHT_BRANCH_L2:
        currentSuperState = CurrentSuperState.AUTO_ALIGN_RIGHT_BRANCH_L2;
        break;
      case AUTO_ALIGN_RIGHT_BRANCH_L3:
        currentSuperState = CurrentSuperState.AUTO_ALIGN_RIGHT_BRANCH_L3;
        break;
      case AUTO_ALIGN_RIGHT_BRANCH_L4:
        currentSuperState = CurrentSuperState.AUTO_ALIGN_RIGHT_BRANCH_L4;
        break;
      case AUTO_ALIGN_MIDDLE_ALGAE:
        currentSuperState = CurrentSuperState.AUTO_ALIGN_MIDDLE_ALGAE;
        break;
      case AUTO_SCORE_L1_LEFT:
        if (rollers.isCoralTroughScored()) {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
          wantedSuperState = WantedSuperState.INTAKING_CORAL_STATION;
        } else if (drivetrain.isAtDriveToPointSetpoints()
            && wrist.isAtSetpoint(WristStates.POSITION_TROUGH)
            && elevator.isAtSetpoint(ElevatorStates.POSITION_CORAL_L1)
            && (currentSuperState == CurrentSuperState.POSITION_CORAL_L1
                || currentSuperState == CurrentSuperState.SCORING_CORAL_TROUGH)) {
          currentSuperState = CurrentSuperState.SCORING_CORAL_TROUGH;
        } else if (drivetrain.isReadyToRaiseAutoScoreTroughCoral()
            && rollers.isCoralEnsured()
            && (currentSuperState == CurrentSuperState.AUTO_ALIGN_LEFT_TROUGH
                || currentSuperState == CurrentSuperState.POSITION_CORAL_L1)) {
          currentSuperState = CurrentSuperState.POSITION_CORAL_L1;
        } else {
          currentSuperState = CurrentSuperState.AUTO_ALIGN_LEFT_TROUGH;
        }
        break;
      case AUTO_SCORE_L1_MIDDLE_LEFT:
        if (rollers.isCoralTroughScored()) {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
          wantedSuperState = WantedSuperState.INTAKING_CORAL_STATION;
        } else if (drivetrain.isAtDriveToPointSetpoints()
            && wrist.isAtSetpoint(WristStates.POSITION_TROUGH)
            && elevator.isAtSetpoint(ElevatorStates.POSITION_CORAL_L1)
            && (currentSuperState == CurrentSuperState.POSITION_CORAL_L1
                || currentSuperState == CurrentSuperState.SCORING_CORAL_TROUGH)) {
          currentSuperState = CurrentSuperState.SCORING_CORAL_TROUGH;
        } else if (drivetrain.isReadyToRaiseAutoScoreTroughCoral()
            && rollers.isCoralEnsured()
            && (currentSuperState == CurrentSuperState.AUTO_ALIGN_MIDDLE_LEFT_TROUGH
                || currentSuperState == CurrentSuperState.POSITION_CORAL_L1)) {
          currentSuperState = CurrentSuperState.POSITION_CORAL_L1;
        } else {
          currentSuperState = CurrentSuperState.AUTO_ALIGN_MIDDLE_LEFT_TROUGH;
        }
        break;
      case AUTO_SCORE_L1_MIDDLE_RIGHT:
        if (rollers.isCoralTroughScored()) {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
          wantedSuperState = WantedSuperState.INTAKING_CORAL_STATION;
        } else if (drivetrain.isAtDriveToPointSetpoints()
            && wrist.isAtSetpoint(WristStates.POSITION_TROUGH)
            && elevator.isAtSetpoint(ElevatorStates.POSITION_CORAL_L1)
            && (currentSuperState == CurrentSuperState.POSITION_CORAL_L1
                || currentSuperState == CurrentSuperState.SCORING_CORAL_TROUGH)) {
          currentSuperState = CurrentSuperState.SCORING_CORAL_TROUGH;
        } else if (drivetrain.isReadyToRaiseAutoScoreTroughCoral()
            && rollers.isCoralEnsured()
            && (currentSuperState == CurrentSuperState.AUTO_ALIGN_MIDDLE_RIGHT_TROUGH
                || currentSuperState == CurrentSuperState.POSITION_CORAL_L1)) {
          currentSuperState = CurrentSuperState.POSITION_CORAL_L1;
        } else {
          currentSuperState = CurrentSuperState.AUTO_ALIGN_MIDDLE_RIGHT_TROUGH;
        }
        break;
      case AUTO_SCORE_L1_RIGHT:
        if (rollers.isCoralTroughScored()) {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
          wantedSuperState = WantedSuperState.INTAKING_CORAL_STATION;
        } else if (drivetrain.isAtDriveToPointSetpoints()
            && wrist.isAtSetpoint(WristStates.POSITION_TROUGH)
            && elevator.isAtSetpoint(ElevatorStates.POSITION_CORAL_L1)
            && (currentSuperState == CurrentSuperState.POSITION_CORAL_L1
                || currentSuperState == CurrentSuperState.SCORING_CORAL_TROUGH)) {
          currentSuperState = CurrentSuperState.SCORING_CORAL_TROUGH;
        } else if (drivetrain.isReadyToRaiseAutoScoreTroughCoral()
            && rollers.isCoralEnsured()
            && (currentSuperState == CurrentSuperState.AUTO_ALIGN_RIGHT_TROUGH
                || currentSuperState == CurrentSuperState.POSITION_CORAL_L1)) {
          currentSuperState = CurrentSuperState.POSITION_CORAL_L1;
        } else {
          currentSuperState = CurrentSuperState.AUTO_ALIGN_RIGHT_TROUGH;
        }
        break;
      case AUTO_SCORE_L2_LEFT:
        if (rollers.isCoralBranchScored()) {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
          wantedSuperState = WantedSuperState.INTAKING_CORAL_STATION;
        } else if (drivetrain.isAtDriveToPointSetpoints()
            && wrist.isAtSetpoint(WristStates.POSITION_BRANCH_L2)
            && elevator.isAtSetpoint(ElevatorStates.POSITION_CORAL_L2)
            && (currentSuperState == CurrentSuperState.POSITION_CORAL_L2
                || currentSuperState == CurrentSuperState.SCORING_CORAL_L2_L3)) {
          currentSuperState = CurrentSuperState.SCORING_CORAL_L2_L3;
        } else if (drivetrain.isReadyToRaiseAutoScoreL2L3Coral()
            && rollers.isCoralEnsured()
            && (currentSuperState == CurrentSuperState.AUTO_ALIGN_LEFT_BRANCH_L2
                || currentSuperState == CurrentSuperState.POSITION_CORAL_L2)) {
          currentSuperState = CurrentSuperState.POSITION_CORAL_L2;
        } else {
          currentSuperState = CurrentSuperState.AUTO_ALIGN_LEFT_BRANCH_L2;
        }
        break;
      case AUTO_SCORE_L2_RIGHT:
        if (rollers.isCoralBranchScored()) {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
          wantedSuperState = WantedSuperState.INTAKING_CORAL_STATION;
        } else if (drivetrain.isAtDriveToPointSetpoints()
            && wrist.isAtSetpoint(WristStates.POSITION_BRANCH_L2)
            && elevator.isAtSetpoint(ElevatorStates.POSITION_CORAL_L2)
            && (currentSuperState == CurrentSuperState.POSITION_CORAL_L2
                || currentSuperState == CurrentSuperState.SCORING_CORAL_L2_L3)) {
          currentSuperState = CurrentSuperState.SCORING_CORAL_L2_L3;
        } else if (drivetrain.isReadyToRaiseAutoScoreL2L3Coral()
            && rollers.isCoralEnsured()
            && (currentSuperState == CurrentSuperState.AUTO_ALIGN_RIGHT_BRANCH_L2
                || currentSuperState == CurrentSuperState.POSITION_CORAL_L2)) {
          currentSuperState = CurrentSuperState.POSITION_CORAL_L2;
        } else {
          currentSuperState = CurrentSuperState.AUTO_ALIGN_RIGHT_BRANCH_L2;
        }
        break;
      case AUTO_SCORE_L3_LEFT:
        if (rollers.isCoralBranchScored()) {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
          wantedSuperState = WantedSuperState.INTAKING_CORAL_STATION;
        } else if (drivetrain.isAtDriveToPointSetpoints()
            && wrist.isAtSetpoint(WristStates.POSITION_BRANCH_L3)
            && elevator.isAtSetpoint(ElevatorStates.POSITION_CORAL_L3)
            && (currentSuperState == CurrentSuperState.POSITION_CORAL_L3
                || currentSuperState == CurrentSuperState.SCORING_CORAL_L2_L3)) {
          currentSuperState = CurrentSuperState.SCORING_CORAL_L2_L3;
        } else if (drivetrain.isReadyToRaiseAutoScoreL2L3Coral()
            && rollers.isCoralEnsured()
            && (currentSuperState == CurrentSuperState.AUTO_ALIGN_LEFT_BRANCH_L3
                || currentSuperState == CurrentSuperState.POSITION_CORAL_L3)) {
          currentSuperState = CurrentSuperState.POSITION_CORAL_L3;
        } else {
          currentSuperState = CurrentSuperState.AUTO_ALIGN_LEFT_BRANCH_L3;
        }
        break;
      case AUTO_SCORE_L3_RIGHT:
        if (rollers.isCoralBranchScored()) {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
          wantedSuperState = WantedSuperState.INTAKING_CORAL_STATION;
        } else if (drivetrain.isAtDriveToPointSetpoints()
            && wrist.isAtSetpoint(WristStates.POSITION_BRANCH_L3)
            && elevator.isAtSetpoint(ElevatorStates.POSITION_CORAL_L3)
            && (currentSuperState == CurrentSuperState.POSITION_CORAL_L3
                || currentSuperState == CurrentSuperState.SCORING_CORAL_L2_L3)) {
          currentSuperState = CurrentSuperState.SCORING_CORAL_L2_L3;
        } else if (drivetrain.isReadyToRaiseAutoScoreL2L3Coral()
            && rollers.isCoralEnsured()
            && (currentSuperState == CurrentSuperState.AUTO_ALIGN_RIGHT_BRANCH_L3
                || currentSuperState == CurrentSuperState.POSITION_CORAL_L3)) {
          currentSuperState = CurrentSuperState.POSITION_CORAL_L3;
        } else {
          currentSuperState = CurrentSuperState.AUTO_ALIGN_RIGHT_BRANCH_L3;
        }
        break;
      case AUTO_SCORE_L4_LEFT:
        if (rollers.isCoralBranchScored()) {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
          wantedSuperState = WantedSuperState.INTAKING_CORAL_STATION;
        } else if (drivetrain.isAtDriveToPointSetpoints()
            && wrist.isAtSetpoint(WristStates.POSITION_BRANCH_L4)
            && elevator.isAtSetpoint(ElevatorStates.POSITION_CORAL_L4)
            && (currentSuperState == CurrentSuperState.POSITION_CORAL_L4
                || currentSuperState == CurrentSuperState.SCORING_CORAL_L4)) {
          currentSuperState = CurrentSuperState.SCORING_CORAL_L4;
        } else if (drivetrain.isReadyToRaiseAutoScoreL4Coral()
            && rollers.isCoralEnsured()
            && (currentSuperState == CurrentSuperState.AUTO_ALIGN_LEFT_BRANCH_L4
                || currentSuperState == CurrentSuperState.POSITION_CORAL_L4)) {
          currentSuperState = CurrentSuperState.POSITION_CORAL_L4;
        } else {
          currentSuperState = CurrentSuperState.AUTO_ALIGN_LEFT_BRANCH_L4;
        }
        break;
      case AUTO_SCORE_L4_RIGHT:
        if (rollers.isCoralBranchScored()) {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
          wantedSuperState = WantedSuperState.INTAKING_CORAL_STATION;
        } else if (drivetrain.isAtDriveToPointSetpoints()
            && wrist.isAtSetpoint(WristStates.POSITION_BRANCH_L4)
            && elevator.isAtSetpoint(ElevatorStates.POSITION_CORAL_L4)
            && (currentSuperState == CurrentSuperState.POSITION_CORAL_L4
                || currentSuperState == CurrentSuperState.SCORING_CORAL_L4)) {
          currentSuperState = CurrentSuperState.SCORING_CORAL_L4;
        } else if (drivetrain.isReadyToRaiseAutoScoreL4Coral()
            && rollers.isCoralEnsured()
            && (currentSuperState == CurrentSuperState.AUTO_ALIGN_RIGHT_BRANCH_L4
                || currentSuperState == CurrentSuperState.POSITION_CORAL_L4)) {
          currentSuperState = CurrentSuperState.POSITION_CORAL_L4;
        } else {
          currentSuperState = CurrentSuperState.AUTO_ALIGN_RIGHT_BRANCH_L4;
        }
        break;
      case AUTO_SCORE_L4_LEFT_AUTO:
        if (rollers.isCoralBranchScored()) {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
          wantedSuperState = WantedSuperState.INTAKING_CORAL_STATION;
        } else if (drivetrain.isAtDriveToPointSetpoints()
            && wrist.isAtSetpoint(WristStates.POSITION_BRANCH_L4)
            && elevator.isAtSetpoint(ElevatorStates.POSITION_CORAL_L4_AUTO)
            && (currentSuperState == CurrentSuperState.POSITION_CORAL_L4_AUTO
                || currentSuperState == CurrentSuperState.SCORING_CORAL_L4)) {
          currentSuperState = CurrentSuperState.SCORING_CORAL_L4;
        } else if (drivetrain.isReadyToRaiseAutoScoreL4Coral()
            && rollers.isCoralEnsured()
            && (currentSuperState == CurrentSuperState.AUTO_ALIGN_LEFT_BRANCH_L4_AUTO
                || currentSuperState == CurrentSuperState.POSITION_CORAL_L4_AUTO)) {
          currentSuperState = CurrentSuperState.POSITION_CORAL_L4_AUTO;
        } else {
          currentSuperState = CurrentSuperState.AUTO_ALIGN_LEFT_BRANCH_L4_AUTO;
        }
        break;
      case AUTO_SCORE_L4_RIGHT_AUTO:
        if (rollers.isCoralBranchScored()) {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
          wantedSuperState = WantedSuperState.INTAKING_CORAL_STATION;
        } else if (drivetrain.isAtDriveToPointSetpoints()
            && wrist.isAtSetpoint(WristStates.POSITION_BRANCH_L4)
            && elevator.isAtSetpoint(ElevatorStates.POSITION_CORAL_L4_AUTO)
            && (currentSuperState == CurrentSuperState.POSITION_CORAL_L4_AUTO
                || currentSuperState == CurrentSuperState.SCORING_CORAL_L4)) {
          currentSuperState = CurrentSuperState.SCORING_CORAL_L4;
        } else if (drivetrain.isReadyToRaiseAutoScoreL4Coral()
            && rollers.isCoralEnsured()
            && (currentSuperState == CurrentSuperState.AUTO_ALIGN_RIGHT_BRANCH_L4_AUTO
                || currentSuperState == CurrentSuperState.POSITION_CORAL_L4_AUTO)) {
          currentSuperState = CurrentSuperState.POSITION_CORAL_L4_AUTO;
        } else {
          currentSuperState = CurrentSuperState.AUTO_ALIGN_RIGHT_BRANCH_L4_AUTO;
        }
        break;
      case POSITION_ALGAE_PROCESSOR:
        if (currentSuperState == CurrentSuperState.POSITION_ALGAE_L2) {
          if (drivetrain.isWithinReefZone() && !rollers.isAlgaeScored()) {
            currentSuperState = CurrentSuperState.POSITION_ALGAE_L2;
            break;
          } else {
            currentSuperState = CurrentSuperState.POSITION_ALGAE_PROCESSOR;
            break;
          }
        } else if (currentSuperState == CurrentSuperState.POSITION_ALGAE_L3) {
          if (drivetrain.isWithinReefZone() && !rollers.isAlgaeScored()) {
            currentSuperState = CurrentSuperState.POSITION_ALGAE_L3;
            break;
          } else {
            currentSuperState = CurrentSuperState.POSITION_ALGAE_PROCESSOR;
            break;
          }
        }
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
        if (rollers.isCoralTroughScored()
            && currentSuperState == CurrentSuperState.SCORING_CORAL_TROUGH) {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
          wantedSuperState = WantedSuperState.INTAKING_CORAL_STATION;
        } else if (rollers.isCoralBranchScored()
            && (currentSuperState == CurrentSuperState.SCORING_CORAL_L2_L3
                || currentSuperState == CurrentSuperState.SCORING_CORAL_L4)) {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
          wantedSuperState = WantedSuperState.INTAKING_CORAL_STATION;
        } else {
          if (currentSuperState == CurrentSuperState.POSITION_CORAL_L1
              || currentSuperState == CurrentSuperState.SCORING_CORAL_TROUGH) {
            currentSuperState = CurrentSuperState.SCORING_CORAL_TROUGH;
          } else if (currentSuperState == CurrentSuperState.POSITION_CORAL_L2
              || currentSuperState == CurrentSuperState.POSITION_CORAL_L3
              || currentSuperState == CurrentSuperState.SCORING_CORAL_L2_L3) {
            currentSuperState = CurrentSuperState.SCORING_CORAL_L2_L3;
          } else if (currentSuperState == CurrentSuperState.POSITION_CORAL_L4
              || currentSuperState == CurrentSuperState.SCORING_CORAL_L4) {
            currentSuperState = CurrentSuperState.SCORING_CORAL_L4;
          } else {
            currentSuperState = CurrentSuperState.SCORING_CORAL_L4;
          }
        }
        break;
      case SCORING_ALGAE:
        if (rollers.isAlgaeScored()) {
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
  }

  private void applyStates() {
    switch (currentSuperState) {
      case INTAKING_CORAL_STATION:
        intakeCoralStation();
        break;
      case INTAKING_ALGAE_GROUND:
        intakeAlgaeGround();
        break;
      case INTAKING_ALGAE_LOLLIPOP:
        intakeAlgaeLollipop();
        break;
      case POSITION_ALGAE_L2:
        positionAlgaeL2();
        break;
      case INTAKING_ALGAE_L2:
        intakeAlgaeL2();
        break;
      case POSITION_ALGAE_L3:
        positionAlgaeL3();
        break;
      case INTAKING_ALGAE_L3:
        intakeAlgaeL3();
        break;
      case POSITION_PREPARED:
        prepare();
        break;
      case POSITION_PREPARED_AUTO:
        prepareInAuto();
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
      case POSITION_CORAL_L4_AUTO:
        positionToCoralL4Auto();
        break;
      case AUTO_ALIGN_LEFT_BRANCH_L2:
        autoAlignToL2orL3Branch(true);
        break;
      case AUTO_ALIGN_LEFT_BRANCH_L3:
        autoAlignToL2orL3Branch(true);
        break;
      case AUTO_ALIGN_LEFT_BRANCH_L4:
        autoAlignToL4Branch(true);
        break;
      case AUTO_ALIGN_LEFT_BRANCH_L4_AUTO:
        autoAlignToL4BranchAuto(true);
        break;
      case AUTO_ALIGN_LEFT_TROUGH:
        autoAlignToTrough(1);
        break;
      case AUTO_ALIGN_MIDDLE_LEFT_TROUGH:
        autoAlignToTrough(2);
        break;
      case AUTO_ALIGN_MIDDLE_RIGHT_TROUGH:
        autoAlignToTrough(3);
        break;
      case AUTO_ALIGN_RIGHT_BRANCH_L2:
        autoAlignToL2orL3Branch(false);
        break;
      case AUTO_ALIGN_RIGHT_BRANCH_L3:
        autoAlignToL2orL3Branch(false);
        break;
      case AUTO_ALIGN_RIGHT_BRANCH_L4:
        autoAlignToL4Branch(false);
        break;
      case AUTO_ALIGN_RIGHT_BRANCH_L4_AUTO:
        autoAlignToL4BranchAuto(false);
        break;
      case AUTO_ALIGN_RIGHT_TROUGH:
        autoAlignToTrough(4);
        break;
      case AUTO_ALIGN_MIDDLE_ALGAE:
        autoAlignToAlgaeReefFace(false);
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
      case SCORING_CORAL_TROUGH:
        scoreCoralInTrough();
        break;
      case SCORING_CORAL_L2_L3:
        scoreCoralOnL2L3();
        break;
      case SCORING_CORAL_L4:
        scoreCoralOnL4();
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

  private void intakeAlgaeGround() {
    elevator.setWantedState(Elevator.WantedState.INTAKING_ALGAE_GROUND);
    rollers.setWantedState(Rollers.WantedState.INTAKING_ALGAE);
    wrist.setWantedState(Wrist.WantedState.INTAKING_ALGAE_GROUND);
    leds.setCurrentState(LEDs.CurrentState.INTAKING_ALGAE_GROUND);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void intakeAlgaeLollipop() {
    elevator.setWantedState(Elevator.WantedState.INTAKING_ALGAE_LOLLIPOP);
    rollers.setWantedState(Rollers.WantedState.INTAKING_ALGAE);
    wrist.setWantedState(Wrist.WantedState.INTAKING_ALGAE_LOLLIPOP);
    leds.setCurrentState(LEDs.CurrentState.INTAKING_ALGAE_GROUND);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void positionAlgaeL2() {
    elevator.setWantedState(Elevator.WantedState.POSITION_ALGAE_L2);
    rollers.setWantedState(Rollers.WantedState.HOLD_ALGAE);
    wrist.setWantedState(Wrist.WantedState.POSITION_PREPARED);
    if (!rollers.isAlgaeScored()) {
      leds.setCurrentState(LEDs.CurrentState.POSITION_ALGAE_PROCESSOR);
    } else {
      leds.setCurrentState(LEDs.CurrentState.INTAKING_ALGAE_L2);
    }
  }

  private void intakeAlgaeL2() {
    elevator.setWantedState(Elevator.WantedState.POSITION_ALGAE_L2);
    rollers.setWantedState(Rollers.WantedState.INTAKING_ALGAE);
    wrist.setWantedState(Wrist.WantedState.INTAKING_ALGAE_REEF_L2);
    leds.setCurrentState(LEDs.CurrentState.INTAKING_ALGAE_L2);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void positionAlgaeL3() {
    elevator.setWantedState(Elevator.WantedState.POSITION_ALGAE_L3);
    rollers.setWantedState(Rollers.WantedState.HOLD_ALGAE);
    wrist.setWantedState(Wrist.WantedState.POSITION_PREPARED);
    if (!rollers.isAlgaeScored()) {
      leds.setCurrentState(LEDs.CurrentState.POSITION_ALGAE_PROCESSOR);
    } else {
      leds.setCurrentState(LEDs.CurrentState.INTAKING_ALGAE_L3);
    }
  }

  private void intakeAlgaeL3() {
    elevator.setWantedState(Elevator.WantedState.POSITION_ALGAE_L3);
    rollers.setWantedState(Rollers.WantedState.INTAKING_ALGAE);
    wrist.setWantedState(Wrist.WantedState.INTAKING_ALGAE_REEF_L3);
    leds.setCurrentState(LEDs.CurrentState.INTAKING_ALGAE_L3);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void prepare() {
    elevator.setWantedState(Elevator.WantedState.POSITION_PREPARED);
    rollers.setWantedState(Rollers.WantedState.HOLD_CORAL);
    wrist.setWantedState(Wrist.WantedState.POSITION_PREPARED);
    leds.setCurrentState(LEDs.CurrentState.POSITION_PREPARED);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void prepareInAuto() {
    elevator.setWantedState(Elevator.WantedState.POSITION_PREPARED);
    rollers.setWantedState(Rollers.WantedState.HOLD_CORAL);
    wrist.setWantedState(Wrist.WantedState.POSITION_PREPARED);
    leds.setCurrentState(LEDs.CurrentState.POSITION_PREPARED);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void positionToCoralL1() {
    elevator.setWantedState(Elevator.WantedState.POSITION_CORAL_L1);
    rollers.setWantedState(Rollers.WantedState.HOLD_CORAL);
    wrist.setWantedState(Wrist.WantedState.POSITION_TROUGH);
    leds.setCurrentState(LEDs.CurrentState.POSITION_CORAL_L1);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void positionToCoralL2() {
    elevator.setWantedState(Elevator.WantedState.POSITION_CORAL_L2);
    rollers.setWantedState(Rollers.WantedState.HOLD_CORAL);
    wrist.setWantedState(Wrist.WantedState.POSITION_BRANCH_L2);
    leds.setCurrentState(LEDs.CurrentState.POSITION_CORAL_L2);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void positionToCoralL3() {
    elevator.setWantedState(Elevator.WantedState.POSITION_CORAL_L3);
    rollers.setWantedState(Rollers.WantedState.HOLD_CORAL);
    wrist.setWantedState(Wrist.WantedState.POSITION_BRANCH_L3);
    leds.setCurrentState(LEDs.CurrentState.POSITION_CORAL_L3);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void positionToCoralL4() {
    elevator.setWantedState(Elevator.WantedState.POSITION_CORAL_L4);
    rollers.setWantedState(Rollers.WantedState.HOLD_CORAL);
    wrist.setWantedState(Wrist.WantedState.POSITION_BRANCH_L4);
    leds.setCurrentState(LEDs.CurrentState.POSITION_CORAL_L4);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void positionToCoralL4Auto() {
    elevator.setWantedState(Elevator.WantedState.POSITION_CORAL_L4_AUTO);
    rollers.setWantedState(Rollers.WantedState.HOLD_CORAL);
    wrist.setWantedState(Wrist.WantedState.POSITION_BRANCH_L4);
    leds.setCurrentState(LEDs.CurrentState.POSITION_CORAL_L4);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void positionToAlgaeProcessor() {
    elevator.setWantedState(Elevator.WantedState.POSITION_ALGAE_PROCESSOR);
    rollers.setWantedState(Rollers.WantedState.HOLD_ALGAE);
    wrist.setWantedState(Wrist.WantedState.POSITION_PREPARED);
    leds.setCurrentState(LEDs.CurrentState.POSITION_ALGAE_PROCESSOR);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void positionToAlgaeBarge() {
    elevator.setWantedState(Elevator.WantedState.POSITION_ALGAE_BARGE);
    rollers.setWantedState(Rollers.WantedState.HOLD_ALGAE);
    wrist.setWantedState(Wrist.WantedState.SCORING_ALGAE_BARGE);
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
  private void autoAlignToL2orL3Branch(boolean useLeftBranch) {
    drivetrain.setTargetPoseForDriveToPoint(
        AutoAlignPoseGenerator.getNearestL2orL3BranchPosition(drivetrain.getPose(), useLeftBranch));
  }

  /**
   * Automatically drives to a branch
   *
   * @param useLeftBranch true if the target branch is the left branch, false if it is the right
   *     branch
   */
  private void autoAlignToL4Branch(boolean useLeftBranch) {
    drivetrain.setTargetPoseForDriveToPoint(
        AutoAlignPoseGenerator.getNearestL4BranchPosition(drivetrain.getPose(), useLeftBranch));
  }

  /**
   * Automatically drives to a branch
   *
   * @param useLeftBranch true if the target branch is the left branch, false if it is the right
   *     branch
   */
  private void autoAlignToL4BranchAuto(boolean useLeftBranch) {
    drivetrain.setTargetPoseForDriveToPoint(
        AutoAlignPoseGenerator.getNearestBranchPosition(drivetrain.getPose(), useLeftBranch));
  }

  /**
   * Automatically drives to a trough
   *
   * @param useLeftBranch true if the target branch is the left branch, false if it is the right
   *     branch
   */
  private void autoAlignToTrough(int troughIndex) {
    drivetrain.setTargetPoseForDriveToPoint(
        AutoAlignPoseGenerator.getNearestTroughPosition(drivetrain.getPose(), troughIndex));
  }

  /**
   * Automatically drives to a reef face
   *
   * @param shiftBackFromReefFace true if the target pose should be shifted back from the reef face,
   *     false if the target pose should be right up to the reef face
   */
  private void autoAlignToAlgaeReefFace(boolean shiftBackFromReefFace) {
    drivetrain.setTargetPoseForDriveToPoint(
        AutoAlignPoseGenerator.getNearestAlgaeReefFacePosition(drivetrain.getPose()));
  }

  private void scoreCoralInTrough() {
    rollers.setWantedState(Rollers.WantedState.SCORING_CORAL_TROUGH);
    leds.setCurrentState(LEDs.CurrentState.SCORING_CORAL);
  }

  private void scoreCoralOnL2L3() {
    rollers.setWantedState(Rollers.WantedState.SCORING_CORAL_L2_L3);
    leds.setCurrentState(LEDs.CurrentState.SCORING_CORAL);
  }

  private void scoreCoralOnL4() {
    rollers.setWantedState(Rollers.WantedState.SCORING_CORAL_L4);
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

  public Command setTeleopDriveStateCommand() {
    return this.runOnce(() -> setTeleopDriveState());
  }

  public void setTeleopDriveState() {
    drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.TELEOP_DRIVE);
    switch (currentSuperState) {
      case AUTO_ALIGN_LEFT_BRANCH_L2:
        wantedSuperState = WantedSuperState.POSITION_PREPARED;
        break;
      case AUTO_ALIGN_LEFT_BRANCH_L3:
        wantedSuperState = WantedSuperState.POSITION_PREPARED;
        break;
      case AUTO_ALIGN_LEFT_BRANCH_L4:
        wantedSuperState = WantedSuperState.POSITION_PREPARED;
        break;
      case AUTO_ALIGN_LEFT_TROUGH:
        wantedSuperState = WantedSuperState.POSITION_PREPARED;
        break;
      case AUTO_ALIGN_MIDDLE_LEFT_TROUGH:
        wantedSuperState = WantedSuperState.POSITION_PREPARED;
        break;
      case AUTO_ALIGN_MIDDLE_RIGHT_TROUGH:
        wantedSuperState = WantedSuperState.POSITION_PREPARED;
        break;
      case AUTO_ALIGN_RIGHT_TROUGH:
        wantedSuperState = WantedSuperState.POSITION_PREPARED;
        break;
      case AUTO_ALIGN_RIGHT_BRANCH_L2:
        wantedSuperState = WantedSuperState.POSITION_PREPARED;
        break;
      case AUTO_ALIGN_RIGHT_BRANCH_L3:
        wantedSuperState = WantedSuperState.POSITION_PREPARED;
        break;
      case AUTO_ALIGN_RIGHT_BRANCH_L4:
        wantedSuperState = WantedSuperState.POSITION_PREPARED;
        break;
      case AUTO_ALIGN_MIDDLE_ALGAE:
        wantedSuperState = WantedSuperState.POSITION_PREPARED;
        break;
      case POSITION_CORAL_L1:
        wantedSuperState = WantedSuperState.POSITION_PREPARED;
        break;
      case POSITION_CORAL_L2:
        wantedSuperState = WantedSuperState.POSITION_PREPARED;
        break;
      case POSITION_CORAL_L3:
        wantedSuperState = WantedSuperState.POSITION_PREPARED;
        break;
      case POSITION_CORAL_L4:
        wantedSuperState = WantedSuperState.POSITION_PREPARED;
        break;
      case POSITION_ALGAE_L2:
        if (wantedSuperState == WantedSuperState.POSITION_ALGAE_PROCESSOR) {
          wantedSuperState = WantedSuperState.POSITION_ALGAE_PROCESSOR;
        } else {
          wantedSuperState = WantedSuperState.POSITION_ALGAE_L2;
        }
        break;
      case INTAKING_ALGAE_L2:
        wantedSuperState = WantedSuperState.POSITION_ALGAE_L2;
        break;
      case POSITION_ALGAE_L3:
        if (wantedSuperState == WantedSuperState.POSITION_ALGAE_PROCESSOR) {
          wantedSuperState = WantedSuperState.POSITION_ALGAE_PROCESSOR;
        } else {
          wantedSuperState = WantedSuperState.POSITION_ALGAE_L3;
        }
        break;
      case INTAKING_ALGAE_L3:
        wantedSuperState = WantedSuperState.POSITION_ALGAE_L3;
        break;
      case SCORING_CORAL_TROUGH:
        wantedSuperState = WantedSuperState.POSITION_PREPARED;
        break;
      case SCORING_CORAL_L2_L3:
        wantedSuperState = WantedSuperState.POSITION_PREPARED;
        break;
      case SCORING_CORAL_L4:
        wantedSuperState = WantedSuperState.POSITION_PREPARED;
        break;
      default:
        break;
    }
  }

  public Command setWantedSuperStateCommand(WantedSuperState wantedState) {
    return runOnce(() -> setWantedSuperState(wantedState));
  }

  public Command setWantedSuperStateInstantCommand(WantedSuperState wantedState) {
    return new InstantCommand(() -> setWantedSuperState(wantedState));
  }

  public void setWantedSuperState(WantedSuperState state) {
    wantedSuperState = state;
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

  public WantedSuperState getWantedSuperState() {
    return wantedSuperState;
  }

  public CurrentSuperState getCurrentSuperState() {
    return currentSuperState;
  }

  /* AUTOS AND BINDINGS ONLY */
  public boolean isCoralEnsured() {
    return rollers.isCoralEnsured();
  }

  public boolean isCoralBranchScored() {
    return rollers.isCoralBranchScored();
  }

  public boolean isAlgaeIntaked() {
    return rollers.isAlgaeIntaked();
  }

  public Command setCoralStateSimCommand(boolean state) {
    return this.runOnce(() -> rollers.setCoralStateSim(state));
  }

  /* TUNING SWERVE ONLY */
  public Command setVelocity(double velocity) {
    return this.runOnce(() -> drivetrain.setVelocity(velocity));
  }
}
