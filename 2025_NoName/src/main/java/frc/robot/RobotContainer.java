package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.algaegroundintake.intakeRollers.IntakeRollers;
import frc.robot.subsystems.algaegroundintake.intakeRollers.IntakeRollersConstants;
import frc.robot.subsystems.algaegroundintake.intakeRollers.IntakeRollersIOSim;
import frc.robot.subsystems.algaegroundintake.intakeRollers.IntakeRollersIOTalonFX;
import frc.robot.subsystems.algaegroundintake.intakewrist.IntakeWrist;
import frc.robot.subsystems.algaegroundintake.intakewrist.IntakeWristConstants.AlgaeStates;
import frc.robot.subsystems.algaegroundintake.intakewrist.IntakeWristIOSim;
import frc.robot.subsystems.algaegroundintake.intakewrist.IntakeWristIOTalonFX;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants.ClimbStates;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.endefector.endefectorrollers.Rollers;
import frc.robot.subsystems.endefector.endefectorrollers.RollersConstants;
import frc.robot.subsystems.endefector.endefectorrollers.RollersIOSim;
import frc.robot.subsystems.endefector.endefectorrollers.RollersIOTalonFX;
import frc.robot.subsystems.endefector.endefectorwrist.Wrist;
import frc.robot.subsystems.endefector.endefectorwrist.WristConstants;
import frc.robot.subsystems.endefector.endefectorwrist.WristIOSim;
import frc.robot.subsystems.endefector.endefectorwrist.WristIOTalonFX;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.subsystemvisualizer.SubsystemVisualizer;

public class RobotContainer {

  private final CommandXboxController driver =
      new CommandXboxController(Constants.ControllerConstants.kDriverControllerPort);
  private final CommandXboxController operator =
      new CommandXboxController(Constants.ControllerConstants.kOperatorControllerPort);

  // visual stuff
  private LEDs LEDs;
  private SubsystemVisualizer subsystemVisualizer;
  private IntakeRollers algaeRollers;
  private IntakeWrist algaeWrist;
  private Elevator elevator;
  private Rollers endefectorRollers;
  private Wrist endefectorWrist;
  private Climb climb;

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        algaeRollers = new IntakeRollers(new IntakeRollersIOTalonFX());
        algaeWrist = new IntakeWrist(new IntakeWristIOTalonFX());
        elevator = new Elevator(new ElevatorIOTalonFX());
        endefectorRollers = new Rollers(new RollersIOTalonFX());
        endefectorWrist = new Wrist(new WristIOTalonFX());
        climb = new Climb(new ClimbIOTalonFX());
        break;
      case SIM:
        algaeRollers = new IntakeRollers(new IntakeRollersIOSim());
        algaeWrist = new IntakeWrist(new IntakeWristIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        endefectorRollers = new Rollers(new RollersIOSim());
        endefectorWrist = new Wrist(new WristIOSim());
        climb = new Climb(new ClimbIOSim());
        break;
    }
    subsystemVisualizer =
        new SubsystemVisualizer(
            elevator, climb, algaeWrist, algaeRollers, endefectorWrist, endefectorRollers);
    DogLog.setOptions(
        new DogLogOptions().withCaptureDs(true).withCaptureNt(true).withNtPublish(true));
    configureBindings();
  }

  private void configureBindings() {
    driver.a().whileTrue(elevator.moveToState(ElevatorConstants.ElevatorStates.L1));

    driver.b().whileTrue(elevator.moveToState(ElevatorConstants.ElevatorStates.L2));

    driver.y().whileTrue(elevator.moveToState(ElevatorConstants.ElevatorStates.L3));

    driver.x().whileTrue(elevator.moveToState(ElevatorConstants.ElevatorStates.L4));

    driver
        .rightTrigger()
        .whileTrue(endefectorRollers.moveToState(RollersConstants.EndefectorRollerStates.SCORE));

    driver
        .rightBumper()
        .whileTrue(endefectorRollers.moveToState(RollersConstants.EndefectorRollerStates.STOP));

    driver
        .leftTrigger()
        .whileTrue(endefectorRollers.moveToState(RollersConstants.EndefectorRollerStates.INTAKE));
  }

  public Command stowAll() {
    return Commands.parallel(
        elevator.moveToState(ElevatorStates.STOW),
        endefectorWrist.moveToState(WristConstants.WristStates.STOW),
        algaeWrist.moveToState(AlgaeStates.STOW),
        climb.moveToState(ClimbStates.STOW),
        algaeRollers.moveToState(IntakeRollersConstants.AlgaeRollerStates.STOW),
        endefectorRollers.moveToState(RollersConstants.EndefectorRollerStates.STOP));
  }

  public Command climbPrep() {
    return climb.moveToState(ClimbStates.CLIMBREADY);
  }

  public Command climbStow() {
    return climb.moveToState(ClimbStates.STOW);
  }

  public Command climbFull() {
    return climb.moveToState(ClimbStates.CLIMB);
  }

  public Command score() {
    return Commands.sequence(
        endefectorWrist.moveToState(WristConstants.WristStates.SCORING),
        endefectorRollers.moveToState(RollersConstants.EndefectorRollerStates.SCORE));
  }

  public Command stationIntake() {
    return Commands.sequence(
        endefectorWrist.moveToState(WristConstants.WristStates.STATIONINTAKE),
        endefectorRollers.moveToState(RollersConstants.EndefectorRollerStates.INTAKE));
  }

  public Command groundIntakeDeploy() {
    return Commands.sequence(
        algaeWrist.moveToState(AlgaeStates.DEPLOYED),
        algaeRollers.moveToState(IntakeRollersConstants.AlgaeRollerStates.INTAKE));
  }

  public Command groundIntakeStow() {
    return Commands.sequence(
        algaeWrist.moveToState(AlgaeStates.STOW),
        algaeRollers.moveToState(IntakeRollersConstants.AlgaeRollerStates.STOW));
  }

  public Command rumbleControllers() {
    return new StartEndCommand(
            () -> driver.getHID().setRumble(RumbleType.kBothRumble, 1),
            () -> driver.getHID().setRumble(RumbleType.kBothRumble, 0))
        .alongWith(
            new StartEndCommand(
                () -> operator.getHID().setRumble(RumbleType.kBothRumble, 1),
                () -> operator.getHID().setRumble(RumbleType.kBothRumble, 0)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
