package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SuperstructureCommands;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class AutoRoutines {
  private AutoFactory autoFactory;
  private SuperstructureCommands superstructureCommands;

  public AutoRoutines(AutoFactory autoFactory, SuperstructureCommands superstructureCommands) {
    this.autoFactory = autoFactory;
    this.superstructureCommands = superstructureCommands;
  }

  public AutoRoutine testingAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("testingAuto");

    // Load the routine's trajectories
    AutoTrajectory LEFTtoI = routine.trajectory("LEFTtoI");
    AutoTrajectory ItoHP = routine.trajectory("ItoHP");
    AutoTrajectory HPtoL = routine.trajectory("HPtoL");
    AutoTrajectory LtoHP = routine.trajectory("LtoHP");
    AutoTrajectory HPtoK = routine.trajectory("HPtoK");

    // When the routine begins, reset odometry and start the first trajectory to go to the reef
    routine.active().onTrue(Commands.sequence(LEFTtoI.resetOdometry(), LEFTtoI.cmd()));

    // // When the previous trajectory is done, score L3, then go to HP and run intake
    // LEFTtoI.done()
    //     .onTrue(
    //         Commands.sequence(
    //             superstructureCommands.scoring(ElevatorConstants.ElevatorStates.L3),
    //             Commands.parallel(ItoHP.cmd(), superstructureCommands.stationIntake())));

    // // When the previous trajectory is done, start the next trajectory to go to the reef
    // ItoHP.done().onTrue(HPtoL.cmd());

    // // When the previous trajectory is done, score L3, then go to HP and run intake
    // HPtoL.done()
    //     .onTrue(
    //         Commands.sequence(
    //             superstructureCommands.scoring(ElevatorConstants.ElevatorStates.L3),
    //             Commands.parallel(LtoHP.cmd(), superstructureCommands.stationIntake())));

    // // When the previous trajectory is done, start the next trajectory to go to the reef
    // LtoHP.done().onTrue(HPtoK.cmd());

    // // When the previous trajectory is done, score L3
    // HPtoK.done().onTrue(superstructureCommands.scoring(ElevatorConstants.ElevatorStates.L3));

    return routine;
  }

  public AutoRoutine leftAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("leftAuto");

    // Load the routine's trajectories
    AutoTrajectory LEFTtoI = routine.trajectory("LEFTtoI");
    AutoTrajectory ItoHP = routine.trajectory("ItoHP");
    AutoTrajectory HPtoL = routine.trajectory("HPtoL");
    AutoTrajectory LtoHP = routine.trajectory("LtoHP");
    AutoTrajectory HPtoK = routine.trajectory("HPtoK");

    // When the routine begins, reset odometry and start the first trajectory to go to the reef
    routine.active().onTrue(Commands.sequence(LEFTtoI.resetOdometry(), LEFTtoI.cmd()));

    // When the previous trajectory is done, score L3, then go to HP and run intake
    LEFTtoI.done()
        .onTrue(
            Commands.sequence(
                superstructureCommands.scoring(ElevatorConstants.ElevatorStates.L3),
                Commands.parallel(ItoHP.cmd(), superstructureCommands.stationIntake())));

    // When the previous trajectory is done, start the next trajectory to go to the reef
    ItoHP.done().onTrue(HPtoL.cmd());

    // When the previous trajectory is done, score L3, then go to HP and run intake
    HPtoL.done()
        .onTrue(
            Commands.sequence(
                superstructureCommands.scoring(ElevatorConstants.ElevatorStates.L3),
                Commands.parallel(LtoHP.cmd(), superstructureCommands.stationIntake())));

    // When the previous trajectory is done, start the next trajectory to go to the reef
    LtoHP.done().onTrue(HPtoK.cmd());

    // When the previous trajectory is done, score L3
    HPtoK.done().onTrue(superstructureCommands.scoring(ElevatorConstants.ElevatorStates.L3));

    return routine;
  }

  public AutoRoutine rightAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("rightAuto");

    // Load the routine's trajectories
    AutoTrajectory RIGHTtoF = routine.trajectory("RIGHTtoF");
    AutoTrajectory FtoHP = routine.trajectory("FtoHP");
    AutoTrajectory HPtoC = routine.trajectory("HPtoC");
    AutoTrajectory CtoHP = routine.trajectory("CtoHP");
    AutoTrajectory HPtoD = routine.trajectory("HPtoD");

    // When the routine begins, reset odometry and start the first trajectory to go to the reef
    routine.active().onTrue(Commands.sequence(RIGHTtoF.resetOdometry(), RIGHTtoF.cmd()));

    // When the previous trajectory is done, score L3, then go to HP and run intake
    RIGHTtoF.done()
        .onTrue(
            Commands.sequence(
                superstructureCommands.scoring(ElevatorConstants.ElevatorStates.L3),
                Commands.parallel(FtoHP.cmd(), superstructureCommands.stationIntake())));

    // When the previous trajectory is done, start the next trajectory to go to the reef
    FtoHP.done().onTrue(HPtoC.cmd());

    // When the previous trajectory is done, score L3, then go to HP and run intake
    HPtoC.done()
        .onTrue(
            Commands.sequence(
                superstructureCommands.scoring(ElevatorConstants.ElevatorStates.L3),
                Commands.parallel(CtoHP.cmd(), superstructureCommands.stationIntake())));

    // When the previous trajectory is done, start the next trajectory to go to the reef
    CtoHP.done().onTrue(HPtoD.cmd());

    // When the previous trajectory is done, score L3
    HPtoD.done().onTrue(superstructureCommands.scoring(ElevatorConstants.ElevatorStates.L3));

    return routine;
  }

  public AutoRoutine middleAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("middleAuto");

    // Load the routine's trajectories
    AutoTrajectory MIDtoG = routine.trajectory("MIDtoG");
    AutoTrajectory GtoS4 = routine.trajectory("GtoS4");
    AutoTrajectory S4toNET = routine.trajectory("S4toNET");

    // When the routine begins, reset odometry and start the first trajectory
    routine.active().onTrue(Commands.sequence(MIDtoG.resetOdometry(), MIDtoG.cmd()));

    // When the previous routine is done, score L3, go to the S4, grab algae, and go to net
    MIDtoG.done()
        .onTrue(
            Commands.sequence(
                superstructureCommands.scoring(ElevatorConstants.ElevatorStates.L3),
                GtoS4.cmd(),
                superstructureCommands.algaeL2Intake(),
                S4toNET.cmd()));

    return routine;
  }

  public AutoRoutine taxiAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("taxiAuto");

    // Load the routine's trajectories
    AutoTrajectory taxi = routine.trajectory("Taxi");

    // When the routine begins, reset odometry and start the first trajectory
    routine.active().onTrue(Commands.sequence(taxi.resetOdometry(), taxi.cmd()));

    return routine;
  }
}
