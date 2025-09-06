package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedSuperState;

public class AutoRoutines {
  private AutoFactory autoFactory;
  private Superstructure superstructureCommands;

  public AutoRoutines(AutoFactory autoFactory, Superstructure superstructureCommands) {
    this.autoFactory = autoFactory;
    this.superstructureCommands = superstructureCommands;
  }

  public AutoRoutine leftAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("leftAuto");

    AutoTrajectory LEFTtoJ = routine.trajectory("LEFTtoJ");
    AutoTrajectory JtoHP = routine.trajectory("JtoHP");
    AutoTrajectory HPtoL = routine.trajectory("HPtoL");
    AutoTrajectory LtoHP = routine.trajectory("LtoHP");
    AutoTrajectory HPtoK = routine.trajectory("HPtoK");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                LEFTtoJ.resetOdometry(),
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.POSITION_PREPARED),
                LEFTtoJ.cmd()));

    // LEFTtoJ.active()
    //     .and(
    //         () ->
    //             (superstructureCommands.isWithinCoralRaiseDistance()
    //                 && superstructureCommands.isCoralEnsured()))
    //     .onTrue(
    //
    // superstructureCommands.setWantedSuperStateCommand(WantedSuperState.POSITION_CORAL_L4));

    LEFTtoJ.done()
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(
                WantedSuperState.AUTO_SCORE_L4_RIGHT));

    LEFTtoJ.recentlyDone()
        .and(() -> !superstructureCommands.isCoralEnsured())
        .onTrue(
            Commands.parallel(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.INTAKING_CORAL_STATION),
                JtoHP.cmd()));

    JtoHP.done().onTrue(Commands.sequence(HPtoL.cmd()));

    // HPtoL.active()
    //     .and(
    //         () ->
    //             (superstructureCommands.isWithinCoralRaiseDistance()
    //                 && superstructureCommands.isCoralEnsured()))
    //     .onTrue(
    //
    // superstructureCommands.setWantedSuperStateCommand(WantedSuperState.POSITION_CORAL_L4));

    HPtoL.done()
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(
                WantedSuperState.AUTO_SCORE_L4_RIGHT));

    HPtoL.recentlyDone()
        .and(() -> !superstructureCommands.isCoralEnsured())
        .onTrue(
            Commands.parallel(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.INTAKING_CORAL_STATION),
                LtoHP.cmd()));

    LtoHP.done().onTrue(Commands.sequence(HPtoK.cmd()));

    // HPtoK.active()
    //     .and(
    //         () ->
    //             (superstructureCommands.isWithinCoralRaiseDistance()
    //                 && superstructureCommands.isCoralEnsured()))
    //     .onTrue(
    //
    // superstructureCommands.setWantedSuperStateCommand(WantedSuperState.POSITION_CORAL_L4));

    HPtoK.done()
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(WantedSuperState.AUTO_SCORE_L4_LEFT));

    return routine;
  }

  public AutoRoutine rightAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("rightAuto");

    AutoTrajectory RIGHTtoE = routine.trajectory("RIGHTtoE");
    AutoTrajectory EtoHP = routine.trajectory("EtoHP");
    AutoTrajectory HPtoC = routine.trajectory("HPtoC");
    AutoTrajectory CtoHP = routine.trajectory("CtoHP");
    AutoTrajectory HPtoD = routine.trajectory("HPtoD");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                RIGHTtoE.resetOdometry(),
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.POSITION_PREPARED),
                superstructureCommands.setCoralStateSimCommand(true),
                RIGHTtoE.cmd()));

    RIGHTtoE.done()
        .onTrue(
            Commands.sequence(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.AUTO_SCORE_L4_LEFT),
                new WaitCommand(0.5),
                superstructureCommands.setCoralStateSimCommand(false)));

    RIGHTtoE.recentlyDone()
        .and(() -> !superstructureCommands.isCoralEnsured())
        .onTrue(
            Commands.sequence(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.INTAKING_CORAL_STATION),
                superstructureCommands.setCoralStateSimCommand(true),
                EtoHP.cmd()));

    EtoHP.done().onTrue(HPtoC.cmd());

    HPtoC.done()
        .onTrue(
            Commands.sequence(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.AUTO_SCORE_L4_LEFT),
                new WaitCommand(0.5),
                superstructureCommands.setCoralStateSimCommand(false)));

    HPtoC.recentlyDone()
        .and(() -> !superstructureCommands.isCoralEnsured())
        .onTrue(
            Commands.sequence(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.INTAKING_CORAL_STATION),
                superstructureCommands.setCoralStateSimCommand(true),
                CtoHP.cmd()));

    CtoHP.done().onTrue(HPtoD.cmd());

    HPtoD.done()
        .onTrue(
            Commands.sequence(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.AUTO_SCORE_L4_RIGHT),
                new WaitCommand(0.5),
                superstructureCommands.setCoralStateSimCommand(false)));

    return routine;
  }

  public AutoRoutine middleAutoAndGrabAlgaeRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("middleAutoAndGrabAlgae");

    AutoTrajectory MIDtoG = routine.trajectory("MIDDLEtoG");
    AutoTrajectory GtoGH = routine.trajectory("GtoGH");

    Trigger withinCoralRaiseDistance =
        new Trigger(() -> superstructureCommands.isWithinCoralRaiseDistance());
    Trigger hasNoCoral = new Trigger(() -> superstructureCommands.isCoralEnsured());

    routine
        .active()
        .onTrue(
            Commands.sequence(
                MIDtoG.resetOdometry(),
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.POSITION_PREPARED),
                MIDtoG.cmd()));

    MIDtoG.active()
        .and(withinCoralRaiseDistance)
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(WantedSuperState.POSITION_CORAL_L4));

    MIDtoG.done()
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(WantedSuperState.AUTO_SCORE_L4_LEFT));

    MIDtoG.recentlyDone().and(hasNoCoral).onTrue(GtoGH.cmd());

    // GtoGH.done()
    //     .onTrue(
    //         superstructureCommands.setWantedSuperStateCommand(
    //             superstructureCommands.returnAutoAlgaeIntakeState()));

    return routine;
  }

  public AutoRoutine middleAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("middleAuto");

    AutoTrajectory MIDtoG = routine.trajectory("MIDDLEtoG");

    Trigger withinCoralRaiseDistance =
        new Trigger(() -> superstructureCommands.isWithinCoralRaiseDistance());

    routine
        .active()
        .onTrue(
            Commands.sequence(
                MIDtoG.resetOdometry(),
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.POSITION_PREPARED),
                MIDtoG.cmd()));

    MIDtoG.active()
        .and(withinCoralRaiseDistance)
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(WantedSuperState.POSITION_CORAL_L4));

    MIDtoG.done()
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(WantedSuperState.AUTO_SCORE_L4_LEFT));

    return routine;
  }

  public AutoRoutine taxiAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("taxiAuto");

    AutoTrajectory MIDtoTaxi = routine.trajectory("MIDDLEtoTaxi");

    routine.active().onTrue(Commands.sequence(MIDtoTaxi.resetOdometry(), MIDtoTaxi.cmd()));

    return routine;
  }

  //   private Command followTrajectoryAndAutoScore(
  //       Trajectory<SwerveSample> trajectory, boolean scoreLeft) {
  //     return (followTrajectory(trajectory)
  //             .andThen(
  //                 new WaitUntilCommand(() -> superstructureCommands.isAtEndOfChoreoTrajectory())
  //                     .andThen(autoScore(scoreLeft))))
  //         .alongWith(
  //             new WaitUntilCommand(() -> superstructureCommands.isWithinCoralRaiseDistance())
  //                 .andThen(
  //                     superstructureCommands.setWantedSuperStateCommand(
  //                         WantedSuperState.POSITION_CORAL_L4)));
  //   }

  //   private Command followTrajectory(Trajectory<SwerveSample> trajectory) {
  //     return new InstantCommand(() ->
  // superstructureCommands.setDesiredChoreoTrajectory(trajectory));
  //   }

  //   private Command autoScore(boolean scoreLeft) {
  //     return superstructureCommands.setWantedSuperStateCommand(
  //         scoreLeft ? WantedSuperState.AUTO_SCORE_L4_LEFT :
  // WantedSuperState.AUTO_SCORE_L4_RIGHT);
  //   }
}
