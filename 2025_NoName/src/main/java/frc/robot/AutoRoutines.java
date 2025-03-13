package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SuperstructureCommands;

public class AutoRoutines {
  private AutoFactory autoFactory;
  private SuperstructureCommands superstructureCommands;

  public AutoRoutines(AutoFactory autoFactory, SuperstructureCommands superstructureCommands) {
    this.autoFactory = autoFactory;
    this.superstructureCommands = superstructureCommands;
  }

  public AutoRoutine testingAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("leftAuto");

    // Load the routine's trajectories
    AutoTrajectory LEFTtoI = routine.trajectory("LEFTtoI");
    AutoTrajectory ItoHP = routine.trajectory("ItoHP");
    AutoTrajectory HPtoL = routine.trajectory("HPtoL");
    AutoTrajectory LtoHP = routine.trajectory("LtoHP");
    AutoTrajectory HPtoK = routine.trajectory("HPtoK");

    // When the routine begins, reset odometry and start the first trajectory to go to the reef
    routine.active().onTrue(Commands.sequence(LEFTtoI.resetOdometry(), LEFTtoI.cmd()));

    // Just before reaching the reef, extend the elevator to L3
    LEFTtoI.atTime("moveToL3").onTrue(superstructureCommands.moveToL3());

    // When the previous trajectory is done, score, then go to HP
    LEFTtoI.done().onTrue(Commands.sequence(superstructureCommands.scoreCoral(), ItoHP.cmd()));

    // // Just before reaching the HP station, start intaking
    // ItoHP.atTime("HPIntake").onTrue(superstructureCommands.stationIntake());

    // // When the previous trajectory is done, start the next trajectory to go to the reef
    // ItoHP.done().onTrue(HPtoL.cmd());

    // // Just before reaching the reef, extend the elevator to L3
    // HPtoL.atTime("moveToL3").onTrue(superstructureCommands.moveToL3());

    // // When the previous trajectory is done, score, then go to HP
    // HPtoL.done()
    //     .onTrue(
    //         Commands.sequence(
    //             superstructureCommands.scoreCoral(),
    //             LtoHP.cmd()));

    // // Just before reaching the HP station, start intaking
    // LtoHP.atTime("HPIntake").onTrue(superstructureCommands.stationIntake());

    // // When the previous trajectory is done, start the next trajectory to go to the reef
    // LtoHP.done().onTrue(HPtoK.cmd());

    // // Just before reaching the reef, extend the elevator to L3
    // HPtoK.atTime("moveToL3").onTrue(superstructureCommands.moveToL3());

    // // When the previous trajectory is done, score L3
    // HPtoK.done()
    //     .onTrue(
    //         Commands.sequence(
    //             superstructureCommands.scoreCoral()));

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

    // Just before reaching the reef, extend the elevator to L3
    LEFTtoI.atTime("moveToL3").onTrue(superstructureCommands.moveToL3());

    // When the previous trajectory is done, score, then go to HP
    LEFTtoI.done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(0.5), superstructureCommands.scoreCoral(), ItoHP.cmd()));

    // Just before reaching the HP station, start intaking
    // ItoHP.atTime("HPIntake").onTrue(superstructureCommands.stationIntake());

    // When the previous trajectory is done, start the next trajectory to go to the reef
    ItoHP.done().onTrue(Commands.sequence(new WaitCommand(1), HPtoL.cmd()));

    // Just before reaching the reef, extend the elevator to L3
    HPtoL.atTime("moveToL3").onTrue(superstructureCommands.moveToL3());

    // When the previous trajectory is done, score, then go to HP
    HPtoL.done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(0.5), superstructureCommands.scoreCoral(), LtoHP.cmd()));

    // Just before reaching the HP station, start intaking
    // LtoHP.atTime("HPIntake").onTrue(superstructureCommands.stationIntake());

    // When the previous trajectory is done, start the next trajectory to go to the reef
    LtoHP.done().onTrue(Commands.sequence(new WaitCommand(1), HPtoK.cmd()));

    // Just before reaching the reef, extend the elevator to L3
    HPtoK.atTime("moveToL3").onTrue(superstructureCommands.moveToL3());

    // When the previous trajectory is done, score L3
    HPtoK.done()
        .onTrue(Commands.sequence(new WaitCommand(0.5), superstructureCommands.scoreCoral()));

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

    // Just before reaching the reef, extend the elevator to L3
    RIGHTtoF.atTime("moveToL3").onTrue(superstructureCommands.moveToL3());

    // When the previous trajectory is done, score, then go to HP
    RIGHTtoF.done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(0.5), superstructureCommands.scoreCoral(), FtoHP.cmd()));

    // Just before reaching the HP station, start intaking
    // FtoHP.atTime("HPIntake").onTrue(superstructureCommands.stationIntake());

    // When the previous trajectory is done, start the next trajectory to go to the reef
    FtoHP.done().onTrue(Commands.sequence(new WaitCommand(1), HPtoC.cmd()));

    // Just before reaching the reef, extend the elevator to L3
    HPtoC.atTime("moveToL3").onTrue(superstructureCommands.moveToL3());

    // When the previous trajectory is done, score L3, then go to HP and run intake
    HPtoC.done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(0.5), superstructureCommands.scoreCoral(), CtoHP.cmd()));

    // Just before reaching the HP station, start intaking
    // CtoHP.atTime("HPIntake").onTrue(superstructureCommands.stationIntake());

    // When the previous trajectory is done, start the next trajectory to go to the reef
    CtoHP.done().onTrue(Commands.sequence(new WaitCommand(1), HPtoD.cmd()));

    // Just before reaching the reef, extend the elevator to L3
    HPtoD.atTime("moveToL3").onTrue(superstructureCommands.moveToL3());

    // When the previous trajectory is done, score L3
    HPtoD.done()
        .onTrue(Commands.sequence(new WaitCommand(0.5), superstructureCommands.scoreCoral()));

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

    // When the previous routine is done, score L3 and go to S4
    MIDtoG.done()
        .onTrue(
            Commands.sequence(
                superstructureCommands.moveToL3(),
                new WaitCommand(0.7),
                superstructureCommands.scoreCoral()));

    // WHen the previous routine is done, grab the algae and go to the net
    // GtoS4.done().onTrue(Commands.sequence(superstructureCommands.algaeL2Intake(),
    // S4toNET.cmd()));

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
