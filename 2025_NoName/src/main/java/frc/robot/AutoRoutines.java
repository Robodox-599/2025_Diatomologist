package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SuperstructureCommands;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;

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

    // When the routine begins, intake, reset odometry, and start the first trajectory to go to the
    // reef
    routine
        .active()
        .onTrue(
            Commands.sequence(
                superstructureCommands.autoIntakeFromStart(),
                LEFTtoI.resetOdometry(),
                LEFTtoI.cmd()));

    // When the previous trajectory is done, move to L4, auto align, score, go to HP, and intake
    LEFTtoI.done()
        .onTrue(
            Commands.sequence(
                superstructureCommands.autoAlignToLeft().withTimeout(1),
                superstructureCommands.moveToL4().withTimeout(1.5),
                superstructureCommands.scoreCoralWithoutIntaking(),
                superstructureCommands.prepareToScore()));

    // // When the previous trajectory is done, wait 1 second, start the next trajectory to go to
    // the reef
    // ItoHP.done().onTrue(Commands.sequence(superstructureCommands.stationIntake()));

    // // When the previous trajectory is done, move to L3, auto align, score, go to HP, and intake
    // HPtoL.done()
    //     .onTrue(
    //         Commands.sequence(
    //             superstructureCommands.moveToL3().withTimeout(1),
    //             superstructureCommands.autoAlignToRight(),
    //             superstructureCommands.scoreCoral(),
    //             LtoHP.cmd(),
    //             superstructureCommands.stationIntake()));

    // // When the previous trajectory is done, wait 1 second, start the next trajectory to go to
    // the reef
    // LtoHP.done().onTrue(Commands.sequence(new WaitCommand(1), HPtoK.cmd()));

    // // When the previous trajectory is done, move to L3, auto align, score, go to HP, and intake
    // HPtoK.done()
    //     .onTrue(
    //         Commands.sequence(
    //             superstructureCommands.moveToL3().withTimeout(1),
    //             superstructureCommands.autoAlignToLeft(),
    //             superstructureCommands.scoreCoral(),
    //             superstructureCommands.stationIntake()));

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
    routine
        .active()
        .onTrue(
            Commands.parallel(
                superstructureCommands.autoIntakeFromStart(),
                Commands.sequence(LEFTtoI.resetOdometry(), LEFTtoI.cmd())));

    // When the previous trajectory is done, move to L4, auto align, score, go to HP, and intake
    LEFTtoI.done()
        .onTrue(
            Commands.sequence(
                // superstructureCommands.autoAlignToLeft(),
                superstructureCommands.moveToL4().withTimeout(1),
                superstructureCommands.scoreCoral(),
                // ItoHP.cmd(),
                superstructureCommands.stationIntake()));

    // When the previous trajectory is done, wait 1 second, start the next trajectory to go to the
    // reef
    ItoHP.done().onTrue(Commands.sequence(new WaitCommand(1), HPtoL.cmd()));

    // When the previous trajectory is done, move to L3, auto align, score, go to HP, and intake
    HPtoL.done()
        .onTrue(
            Commands.sequence(
                superstructureCommands.autoAlignToRight(),
                superstructureCommands.moveToL3().withTimeout(1),
                superstructureCommands.scoreCoral(),
                LtoHP.cmd(),
                superstructureCommands.stationIntake()));

    // When the previous trajectory is done, wait 1 second, start the next trajectory to go to the
    // reef
    LtoHP.done().onTrue(Commands.sequence(new WaitCommand(1), HPtoK.cmd()));

    // When the previous trajectory is done, move to L3, auto align, score, go to HP, and intake
    HPtoK.done()
        .onTrue(
            Commands.sequence(
                superstructureCommands.autoAlignToLeft(),
                superstructureCommands.moveToL3().withTimeout(1),
                superstructureCommands.scoreCoral(),
                superstructureCommands.stationIntake()));

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
    routine
        .active()
        .onTrue(
            Commands.parallel(
                superstructureCommands.autoIntakeFromStart(),
                Commands.sequence(RIGHTtoF.resetOdometry(), RIGHTtoF.cmd())));

    // When the previous trajectory is done, move to L4, auto align, score, go to HP, and intake
    RIGHTtoF.done()
        .onTrue(
            Commands.sequence(
                // superstructureCommands.autoAlignToRight().withTimeout(3),
                superstructureCommands.moveToL4().withTimeout(1),
                superstructureCommands.scoreCoral(),
                // FtoHP.cmd(),
                superstructureCommands.stationIntake()));

    // // When the previous trajectory is done, wait 1 second, start the next trajectory to go to
    // the
    // // reef
    // FtoHP.done().onTrue(Commands.sequence(new WaitCommand(1), HPtoC.cmd()));

    // // When the previous trajectory is done, move to L3, auto align, score, go to HP, and intake
    // HPtoC.done()
    //     .onTrue(
    //         Commands.sequence(
    //             superstructureCommands.autoAlignToLeft(),
    //             superstructureCommands.moveToL3().withTimeout(1),
    //             superstructureCommands.scoreCoral(),
    //             CtoHP.cmd(),
    //             superstructureCommands.stationIntake()));

    // // When the previous trajectory is done, wait 1 second, start the next trajectory to go to
    // the
    // // reef
    // CtoHP.done().onTrue(Commands.sequence(new WaitCommand(1), HPtoD.cmd()));

    // // When the previous trajectory is done, move to L3, auto align, score, go to HP, and intake
    // HPtoD.done()
    //     .onTrue(
    //         Commands.sequence(
    //             superstructureCommands.autoAlignToRight(),
    //             superstructureCommands.moveToL3().withTimeout(1),
    //             superstructureCommands.scoreCoral(),
    //             superstructureCommands.stationIntake()));

    return routine;
  }

  public AutoRoutine middleAutoRoutineWithAlgae() {
    AutoRoutine routine = autoFactory.newRoutine("middleAuto");

    // Load the routine's trajectories
    AutoTrajectory MIDtoG = routine.trajectory("MIDtoG");
    AutoTrajectory GtoS4 = routine.trajectory("GtoS4");
    AutoTrajectory S4toLine = routine.trajectory("S4toLine");

    // When the routine begins, intake, reset odometry, and start the first trajectory
    routine
        .active()
        .onTrue(
            Commands.parallel(
                superstructureCommands.autoIntakeFromStart(),
                Commands.sequence(MIDtoG.resetOdometry(), MIDtoG.cmd())));

    // When the previous trajectory is done, move to L4, auto align, score, go to HP, and intake
    MIDtoG.done()
        .onTrue(
            Commands.sequence(
                // superstructureCommands.autoAlignToLeft().withTimeout(3),
                superstructureCommands.moveToL4().withTimeout(1.5),
                superstructureCommands.scoreCoralWithoutIntaking(),
                GtoS4.cmd()));

    GtoS4.done()
        .onTrue(
            Commands.sequence(
                superstructureCommands.removeAlgae(ElevatorStates.ALGAE_L2), S4toLine.cmd()));

    return routine;
  }

  public AutoRoutine middleAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("middleAuto");

    // Load the routine's trajectories
    AutoTrajectory MIDtoG = routine.trajectory("MIDtoG");

    // When the routine begins, intake, reset odometry, and start the first trajectory
    routine
        .active()
        .onTrue(
            Commands.parallel(
                superstructureCommands.autoIntakeFromStart(),
                Commands.sequence(MIDtoG.resetOdometry(), MIDtoG.cmd())));

    // When the previous trajectory is done, move to L4, auto align, score, go to HP, and intake
    MIDtoG.done()
        .onTrue(
            Commands.sequence(
                // superstructureCommands.autoAlignToLeft().withTimeout(3),
                superstructureCommands.moveToL4().withTimeout(1.5),
                superstructureCommands.scoreCoral()));

    // WHen the previous routine is done, grab the algae and go to the net
    // GtoS4.done().onTrue(Commands.sequence(superstructureCommands.algaeL2Intake(),
    // S4toNET.cmd()));

    return routine;
  }

  public AutoRoutine taxiAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("taxiAuto");

    // Load the routine's trajectories
    AutoTrajectory MIDtoTaxi = routine.trajectory("MIDtoTaxi");

    // When the routine begins, reset odometry and start the first trajectory
    routine.active().onTrue(Commands.sequence(MIDtoTaxi.resetOdometry(), MIDtoTaxi.cmd()));

    return routine;
  }
}
