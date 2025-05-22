package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.SafetyChecker;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.constants.TunerConstants;
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
  private final CommandXboxController operator;
  private final CommandXboxController driver;

  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  // Setting up bindings for necessary control of the swerve drive platform
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(0)
          .withRotationalDeadband(0) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;
  private WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
  private WantedSuperState nextSuperState = WantedSuperState.STOPPED;

  public Superstructure(
      CommandSwerveDrivetrain drivetrain,
      Elevator elevator,
      Wrist wrist,
      Rollers rollers,
      // Climb climb,
      LEDs LEDs,
      SafetyChecker safetyChecker,
      CommandXboxController driver,
      CommandXboxController operator) {
    this.drivetrain = drivetrain;
    this.elevator = elevator;
    this.wrist = wrist;
    this.rollers = rollers;
    this.leds = LEDs;
    this.safetyChecker = safetyChecker;
    this.operator = operator;
    this.driver = driver;
  }

  public enum CurrentSuperState {
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
    STOPPED,
  }

  public enum WantedSuperState {
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
    STOPPED,
  }

  private void setNextState(WantedSuperState state) {
    nextSuperState = state;
  }

  private void updateWantedState() {
    wantedSuperState = nextSuperState;
  }

  private CurrentSuperState handleStateTransitions() {
    switch (wantedSuperState) {
      case INTAKING_CORAL_STATION:
        if (rollers.isCoralDetected()) {
          currentSuperState = CurrentSuperState.PREPARED;
          wantedSuperState = WantedSuperState.PREPARED;
        } else {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
        }
        break;
      case INTAKING_ALGAE_GROUND:
        if (rollers.isAlgaeDetected()) {
          currentSuperState = CurrentSuperState.PREPARED;
          wantedSuperState = WantedSuperState.PREPARED;
        } else {
          currentSuperState = CurrentSuperState.INTAKING_ALGAE_GROUND;
        }
        break;
      case INTAKING_ALGAE_L2:
        if (rollers.isAlgaeDetected()) {
          currentSuperState = CurrentSuperState.PREPARED;
          wantedSuperState = WantedSuperState.PREPARED;
        } else {
          currentSuperState = CurrentSuperState.INTAKING_ALGAE_L2;
        }
        break;
      case INTAKING_ALGAE_L3:
        if (rollers.isAlgaeDetected()) {
          currentSuperState = CurrentSuperState.PREPARED;
          wantedSuperState = WantedSuperState.PREPARED;
        } else {
          currentSuperState = CurrentSuperState.INTAKING_ALGAE_L3;
        }
        break;
      case PREPARED:
        currentSuperState = CurrentSuperState.PREPARED;
        break;
      case SCORING_CORAL_L1:
        if (!rollers.isCoralDetected()) {
          currentSuperState = CurrentSuperState.PREPARED;
          wantedSuperState = WantedSuperState.PREPARED;
        } else {
          currentSuperState = CurrentSuperState.SCORING_CORAL_L1;
        }
        break;
      case SCORING_CORAL_L2:
        if (!rollers.isCoralDetected()) {
          currentSuperState = CurrentSuperState.PREPARED;
          wantedSuperState = WantedSuperState.PREPARED;
        } else {
          currentSuperState = CurrentSuperState.SCORING_CORAL_L2;
        }
        break;
      case SCORING_CORAL_L3:
        if (!rollers.isCoralDetected()) {
          currentSuperState = CurrentSuperState.PREPARED;
          wantedSuperState = WantedSuperState.PREPARED;
        } else {
          currentSuperState = CurrentSuperState.SCORING_CORAL_L3;
        }
        break;
      case SCORING_CORAL_L4:
        if (!rollers.isCoralDetected()) {
          currentSuperState = CurrentSuperState.PREPARED;
          wantedSuperState = WantedSuperState.PREPARED;
        } else {
          currentSuperState = CurrentSuperState.SCORING_CORAL_L4;
        }
        break;
      case SCORING_ALGAE_PROCESSOR:
        if (!rollers.isAlgaeDetected()) {
          currentSuperState = CurrentSuperState.PREPARED;
          wantedSuperState = WantedSuperState.PREPARED;
        } else {
          currentSuperState = CurrentSuperState.SCORING_ALGAE_PROCESSOR;
        }
        break;
      case SCORING_ALGAE_BARGE:
        if (!rollers.isAlgaeDetected()) {
          currentSuperState = CurrentSuperState.PREPARED;
          wantedSuperState = WantedSuperState.PREPARED;
        } else {
          currentSuperState = CurrentSuperState.SCORING_ALGAE_BARGE;
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
      case PREPARED:
        prepare();
        break;
      case SCORING_CORAL_L1:
        scoreCoralL1();
        break;
      case SCORING_CORAL_L2:
        scoreCoralL2();
        break;
      case SCORING_CORAL_L3:
        scoreCoralL3();
        break;
      case SCORING_CORAL_L4:
        scoreCoralL4();
        break;
      case SCORING_ALGAE_PROCESSOR:
        scoreAlgaeProcessor();
        break;
      case SCORING_ALGAE_BARGE:
        scoreAlgaeBarge();
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
    elevator.setWantedState(Elevator.WantedState.PREPARED);
    rollers.setWantedState(Rollers.WantedState.STOPPED);
    wrist.setWantedState(Wrist.WantedState.PREPARED);
    leds.setCurrentState(LEDs.CurrentState.PREPARED);
  }

  private void scoreCoralL1() {
    elevator.setWantedState(Elevator.WantedState.SCORING_CORAL_L1);
    rollers.setWantedState(Rollers.WantedState.SCORING_CORAL);
    wrist.setWantedState(Wrist.WantedState.SCORING_CORAL);
    leds.setCurrentState(LEDs.CurrentState.SCORING_CORAL_L1);
  }

  private void scoreCoralL2() {
    elevator.setWantedState(Elevator.WantedState.SCORING_CORAL_L2);
    rollers.setWantedState(Rollers.WantedState.SCORING_CORAL);
    wrist.setWantedState(Wrist.WantedState.SCORING_CORAL);
    leds.setCurrentState(LEDs.CurrentState.SCORING_CORAL_L2);
  }

  private void scoreCoralL3() {
    elevator.setWantedState(Elevator.WantedState.SCORING_CORAL_L3);
    rollers.setWantedState(Rollers.WantedState.SCORING_CORAL);
    wrist.setWantedState(Wrist.WantedState.SCORING_CORAL);
    leds.setCurrentState(LEDs.CurrentState.SCORING_CORAL_L3);
  }

  private void scoreCoralL4() {
    elevator.setWantedState(Elevator.WantedState.SCORING_CORAL_L4);
    rollers.setWantedState(Rollers.WantedState.SCORING_CORAL);
    wrist.setWantedState(Wrist.WantedState.SCORING_CORAL);
    leds.setCurrentState(LEDs.CurrentState.SCORING_CORAL_L4);
  }

  private void scoreAlgaeProcessor() {
    elevator.setWantedState(Elevator.WantedState.SCORING_ALGAE_PROCESSOR);
    rollers.setWantedState(Rollers.WantedState.SCORING_ALGAE);
    wrist.setWantedState(Wrist.WantedState.SCORING_ALGAE);
    leds.setCurrentState(LEDs.CurrentState.SCORING_ALGAE_PROCESSOR);
  }

  private void scoreAlgaeBarge() {
    elevator.setWantedState(Elevator.WantedState.SCORING_ALGAE_BARGE);
    rollers.setWantedState(Rollers.WantedState.SCORING_ALGAE);
    wrist.setWantedState(Wrist.WantedState.SCORING_ALGAE);
    leds.setCurrentState(LEDs.CurrentState.SCORING_ALGAE_BARGE);
  }

  private void stop() {
    elevator.setWantedState(Elevator.WantedState.STOPPED);
    rollers.setWantedState(Rollers.WantedState.STOPPED);
    wrist.setWantedState(Wrist.WantedState.STOPPED);
    leds.setCurrentState(LEDs.CurrentState.NO_STATE);
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

  public void configureBindings() {
    //                               DRIVER BINDS

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -joystickDeadbandApply(driver.getLeftY())
                            * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -joystickDeadbandApply(driver.getLeftX())
                            * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        joystickDeadbandApply(-driver.getRightX())
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    // driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // driver
    //     .b()
    //     .whileTrue(
    //         drivetrain.applyRequest(
    //             () ->
    //                 point.withModuleDirection(
    //                     new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    // reset the field-centric heading on left bumper press
    // driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    driver.y().onTrue(drivetrain.zeroGyroCommand());
    
    // driver.rightTrigger().onTrue(updateWantedState());

    //                                OPERATOR BINDS
    // // MOVE TO L1
    // operator.x().onTrue(moveToL1());
    // // // MOVE TO L2
    // operator.a().onTrue(moveToL2());
    // // // MOVE TO L3
    // operator.b().onTrue(moveToL3());
    // // // MOVE TO L4
    // operator.y().onTrue(moveToL4());
    // // // MOVE TO NET
    // operator.povLeft().onTrue(extendToNet());
    // operator.povRight().onTrue(extendToNet());
    // // CORAL STATION INTAKE
    // operator.rightBumper().onTrue(coralStationIntake());
    // // PREPARE
    // operator.leftBumper().onTrue(prepareToScore());
    // // // EJECT CORAL
    // operator.leftTrigger().whileTrue(ejectGamePiece());
    // operator.leftTrigger().onFalse(rollers.stop());
    // // // INTAKE ALGAE L2
    // operator.povDown().onTrue(algaeIntake(ElevatorStates.ALGAEL2));
    // // // INTAKE ALGAE L3
    // operator.povUp().onTrue(algaeIntake(ElevatorStates.ALGAEL3));
  }

  private static double joystickDeadbandApply(double x) {
    return MathUtil.applyDeadband(
        (Math.signum(x) * (1.01 * Math.pow(x, 2) - 0.0202 * x + 0.0101)), 0.02);
  }
}
