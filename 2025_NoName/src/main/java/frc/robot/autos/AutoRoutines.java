package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedSuperState;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class AutoRoutines {
  private AutoFactory autoFactory;
  private Superstructure superstructureCommands;
  private CommandSwerveDrivetrain drivetrain;

  public AutoRoutines(
      AutoFactory autoFactory,
      Superstructure superstructureCommands,
      CommandSwerveDrivetrain drivetrain) {
    this.autoFactory = autoFactory;
    this.superstructureCommands = superstructureCommands;
    this.drivetrain = drivetrain;
  }

  public AutoRoutine leftAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("leftAuto");

    AutoTrajectory LEFTtoI = routine.trajectory("LEFTtoI");
    AutoTrajectory ItoHP = routine.trajectory("ItoHP");
    AutoTrajectory HPtoL = routine.trajectory("HPtoL");
    AutoTrajectory LtoHP = routine.trajectory("LtoHP");
    AutoTrajectory HPtoK = routine.trajectory("HPtoK");
    AutoTrajectory KtoHP = routine.trajectory("KtoHP");
    AutoTrajectory HPtoJ = routine.trajectory("HPtoJ");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                LEFTtoI.resetOdometry(),
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.POSITION_PREPARED_AUTO),
                LEFTtoI.cmd()));

    LEFTtoI.active()
        .and(
            () ->
                (drivetrain.isWithinCoralRaiseDistance()
                    && superstructureCommands.isCoralEnsured()))
        .onTrue(
            superstructureCommands.setWantedSuperStateInstantCommand(
                WantedSuperState.POSITION_CORAL_L4));

    LEFTtoI.recentlyDone()
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(WantedSuperState.AUTO_SCORE_L4_LEFT));

    LEFTtoI.recentlyDone()
        .and(() -> !superstructureCommands.isCoralEnsured())
        .onTrue(
            Commands.parallel(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.INTAKING_CORAL_STATION),
                ItoHP.cmd()));

    ItoHP.done().onTrue(Commands.sequence(HPtoL.cmd()));

    HPtoL.active()
        .and(
            () ->
                (drivetrain.isWithinCoralRaiseDistance()
                    && superstructureCommands.isCoralEnsured()))
        .onTrue(
            superstructureCommands.setWantedSuperStateInstantCommand(
                WantedSuperState.POSITION_CORAL_L4));

    HPtoL.recentlyDone()
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

    HPtoK.active()
        .and(
            () ->
                (drivetrain.isWithinCoralRaiseDistance()
                    && superstructureCommands.isCoralEnsured()))
        .onTrue(
            superstructureCommands.setWantedSuperStateInstantCommand(
                WantedSuperState.POSITION_CORAL_L4));

    HPtoK.recentlyDone()
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(WantedSuperState.AUTO_SCORE_L4_LEFT));

    HPtoK.recentlyDone()
        .and(() -> !superstructureCommands.isCoralEnsured())
        .onTrue(
            Commands.parallel(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.INTAKING_CORAL_STATION),
                KtoHP.cmd()));

    KtoHP.done().onTrue(HPtoJ.cmd());

    HPtoJ.active()
        .and(
            () ->
                (drivetrain.isWithinCoralRaiseDistance()
                    && superstructureCommands.isCoralEnsured()))
        .onTrue(
            superstructureCommands.setWantedSuperStateInstantCommand(
                WantedSuperState.POSITION_CORAL_L4));

    HPtoJ.recentlyDone()
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(
                WantedSuperState.AUTO_SCORE_L4_RIGHT));

    return routine;
  }

  public AutoRoutine rightAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("rightAuto");

    AutoTrajectory RIGHTtoF = routine.trajectory("RIGHTtoF");
    AutoTrajectory FtoHP = routine.trajectory("FtoHP");
    AutoTrajectory HPtoC = routine.trajectory("HPtoC");
    AutoTrajectory CtoHP = routine.trajectory("CtoHP");
    AutoTrajectory HPtoD = routine.trajectory("HPtoD");
    AutoTrajectory DtoHP = routine.trajectory("DtoHP");
    AutoTrajectory HPtoE = routine.trajectory("HPtoE");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                RIGHTtoF.resetOdometry(),
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.POSITION_PREPARED),
                superstructureCommands.setCoralStateSimCommand(true),
                RIGHTtoF.cmd()));

    RIGHTtoF.active()
        .and(
            () ->
                (drivetrain.isWithinCoralRaiseDistance()
                    && superstructureCommands.isCoralEnsured()))
        .onTrue(
            superstructureCommands.setWantedSuperStateInstantCommand(
                WantedSuperState.POSITION_CORAL_L4));

    RIGHTtoF.done()
        .onTrue(
            Commands.sequence(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.AUTO_SCORE_L4_RIGHT),
                superstructureCommands.setCoralStateSimCommand(false)));

    RIGHTtoF.recentlyDone()
        .and(() -> !superstructureCommands.isCoralEnsured())
        .onTrue(
            Commands.sequence(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.INTAKING_CORAL_STATION),
                superstructureCommands.setCoralStateSimCommand(true),
                FtoHP.cmd()));

    FtoHP.done().onTrue(HPtoC.cmd());

    HPtoC.done()
        .onTrue(
            Commands.sequence(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.AUTO_SCORE_L4_LEFT),
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
                superstructureCommands.setCoralStateSimCommand(false)));

    HPtoD.recentlyDone()
        .and(() -> !superstructureCommands.isCoralEnsured())
        .onTrue(
            Commands.sequence(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.INTAKING_CORAL_STATION),
                superstructureCommands.setCoralStateSimCommand(true),
                DtoHP.cmd()));

    DtoHP.done().onTrue(HPtoE.cmd());

    HPtoE.done()
        .onTrue(
            Commands.sequence(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.AUTO_SCORE_L4_LEFT),
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
