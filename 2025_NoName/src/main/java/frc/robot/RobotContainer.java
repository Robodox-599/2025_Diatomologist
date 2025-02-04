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
import frc.robot.subsystems.algaegroundintake.intakeWrist.IntakeWrist;
import frc.robot.subsystems.algaegroundintake.intakeWrist.IntakeWristConstants.AlgaeStates;
import frc.robot.subsystems.algaegroundintake.intakeWrist.IntakeWristIOSim;
import frc.robot.subsystems.algaegroundintake.intakeWrist.IntakeWristIOTalonFX;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants.ClimbStates;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.endefector.rollers.Rollers;
import frc.robot.subsystems.endefector.rollers.RollersConstants;
import frc.robot.subsystems.endefector.rollers.RollersConstants.EndefectorRollerStates;
import frc.robot.subsystems.endefector.rollers.RollersIOSim;
import frc.robot.subsystems.endefector.rollers.RollersIOTalonFX;
import frc.robot.subsystems.endefector.wrist.Wrist;
import frc.robot.subsystems.endefector.wrist.WristConstants.WristStates;
import frc.robot.subsystems.endefector.wrist.WristIOSim;
import frc.robot.subsystems.endefector.wrist.WristIOTalonFX;
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
            elevator, climb, LEDs, algaeWrist, algaeRollers, endefectorWrist, endefectorRollers);
    DogLog.setOptions(
        new DogLogOptions().withCaptureDs(true).withCaptureNt(true).withNtPublish(true));
    configureBindings();
  }

  private void configureBindings() {
    // driver.

  }

  public Command stowAll() {
    return Commands.parallel(
        elevator.moveToState(ElevatorStates.STOW),
        endefectorWrist.moveToState(WristStates.STOW),
        algaeWrist.moveToState(AlgaeStates.STOW),
        climb.moveToState(ClimbStates.STOW));
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
        endefectorWrist.moveToState(WristStates.SCORE),
        endefectorRollers.moveToState(EndefectorRollerStates.SCORE));
  }

  public Command stationIntake() {
    return Commands.sequence(
        endefectorWrist.moveToState(WristStates.STATIONINTAKE),
        endefectorRollers.moveToState(RollersConstants.EndefectorRollerStates.INTAKE));
  }

  public Command groundIntakeDeploy() {
    return Commands.sequence(
        algaeWrist.moveToState(AlgaeStates.DEPLOYED),
        algaeRollers.moveToState(IntakeRollersConstants.AlageRollerStates.INTAKE));
  }

  public Command groundIntakeStow() {
    return Commands.sequence(
        algaeWrist.moveToState(AlgaeStates.STOW),
        algaeRollers.moveToState(IntakeRollersConstants.AlageRollerStates.STOP));
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
