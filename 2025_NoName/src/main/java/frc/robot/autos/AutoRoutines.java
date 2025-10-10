package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
            () -> (drivetrain.isWithinL4RaiseDistance() && superstructureCommands.isCoralEnsured()))
        .onTrue(
            superstructureCommands.setWantedSuperStateInstantCommand(
                WantedSuperState.POSITION_CORAL_L4));

    LEFTtoI.recentlyDone()
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(WantedSuperState.AUTO_SCORE_L4_LEFT));

    LEFTtoI.recentlyDone()
        .and(() -> superstructureCommands.isCoralBranchScored())
        .onTrue(
            Commands.parallel(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.INTAKING_CORAL_STATION),
                ItoHP.cmd()));

    ItoHP.done().onTrue(Commands.sequence(new WaitCommand(0.5), HPtoL.cmd()));

    HPtoL.active()
        .and(
            () -> (drivetrain.isWithinL4RaiseDistance() && superstructureCommands.isCoralEnsured()))
        .onTrue(
            superstructureCommands.setWantedSuperStateInstantCommand(
                WantedSuperState.POSITION_CORAL_L4));

    HPtoL.recentlyDone()
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(
                WantedSuperState.AUTO_SCORE_L4_RIGHT));

    HPtoL.recentlyDone()
        .and(() -> superstructureCommands.isCoralBranchScored())
        .onTrue(
            Commands.parallel(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.INTAKING_CORAL_STATION),
                LtoHP.cmd()));

    LtoHP.done().onTrue(Commands.sequence(new WaitCommand(0.5), HPtoK.cmd()));

    HPtoK.active()
        .and(
            () -> (drivetrain.isWithinL4RaiseDistance() && superstructureCommands.isCoralEnsured()))
        .onTrue(
            superstructureCommands.setWantedSuperStateInstantCommand(
                WantedSuperState.POSITION_CORAL_L4));

    HPtoK.recentlyDone()
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(WantedSuperState.AUTO_SCORE_L4_LEFT));

    HPtoK.recentlyDone()
        .and(() -> superstructureCommands.isCoralBranchScored())
        .onTrue(
            Commands.parallel(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.INTAKING_CORAL_STATION),
                KtoHP.cmd()));

    KtoHP.done().onTrue(Commands.sequence(new WaitCommand(0.5), HPtoJ.cmd()));

    HPtoJ.active()
        .and(
            () -> (drivetrain.isWithinL4RaiseDistance() && superstructureCommands.isCoralEnsured()))
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
            () -> (drivetrain.isWithinL4RaiseDistance() && superstructureCommands.isCoralEnsured()))
        .onTrue(
            superstructureCommands.setWantedSuperStateInstantCommand(
                WantedSuperState.POSITION_CORAL_L4_AUTO));

    RIGHTtoF.done()
        .onTrue(
            Commands.sequence(
                Commands.deadline(
                    superstructureCommands.setWantedSuperStateCommand(
                        WantedSuperState.AUTO_SCORE_L4_RIGHT_AUTO),
                    new WaitCommand(0.5)),
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

    HPtoC.active()
        .and(
            () -> (drivetrain.isWithinL4RaiseDistance() && superstructureCommands.isCoralEnsured()))
        .onTrue(
            superstructureCommands.setWantedSuperStateInstantCommand(
                WantedSuperState.POSITION_CORAL_L4_AUTO));

    HPtoC.done()
        .onTrue(
            Commands.sequence(
                Commands.deadline(
                    superstructureCommands.setWantedSuperStateCommand(
                        WantedSuperState.AUTO_SCORE_L4_LEFT_AUTO),
                    new WaitCommand(0.5)),
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

    HPtoD.active()
        .and(
            () -> (drivetrain.isWithinL4RaiseDistance() && superstructureCommands.isCoralEnsured()))
        .onTrue(
            superstructureCommands.setWantedSuperStateInstantCommand(
                WantedSuperState.POSITION_CORAL_L4_AUTO));

    HPtoD.done()
        .onTrue(
            Commands.sequence(
                Commands.deadline(
                    superstructureCommands.setWantedSuperStateCommand(
                        WantedSuperState.AUTO_SCORE_L4_RIGHT_AUTO),
                    new WaitCommand(0.5)),
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

    HPtoE.active()
        .and(
            () -> (drivetrain.isWithinL4RaiseDistance() && superstructureCommands.isCoralEnsured()))
        .onTrue(
            superstructureCommands.setWantedSuperStateInstantCommand(
                WantedSuperState.POSITION_CORAL_L4_AUTO));

    HPtoE.done()
        .onTrue(
            Commands.sequence(
                Commands.deadline(
                    superstructureCommands.setWantedSuperStateCommand(
                        WantedSuperState.AUTO_SCORE_L4_LEFT_AUTO),
                    new WaitCommand(1)),
                superstructureCommands.setCoralStateSimCommand(false)));

    return routine;
  }

  public AutoRoutine middleAutoAndGrabAlgaeRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("middleAutoAndGrabAlgae");

    AutoTrajectory MIDtoG = routine.trajectory("MIDDLEtoG");
    AutoTrajectory GtoGH = routine.trajectory("GtoGH");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                MIDtoG.resetOdometry(),
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.POSITION_PREPARED),
                MIDtoG.cmd()));

    MIDtoG.active()
        .and(
            () -> (drivetrain.isWithinL4RaiseDistance() && superstructureCommands.isCoralEnsured()))
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(
                WantedSuperState.POSITION_CORAL_L4_AUTO));

    MIDtoG.done()
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(
                WantedSuperState.AUTO_SCORE_L4_LEFT_AUTO));

    // MIDtoG.recentlyDone().and(hasNoCoral).onTrue(GtoGH.cmd());

    // GtoGH.done()
    //     .onTrue(
    //         superstructureCommands.setWantedSuperStateCommand(
    //             superstructureCommands.returnAutoAlgaeIntakeState()));

    return routine;
  }

  public AutoRoutine middleAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("middleAuto");

    AutoTrajectory MIDtoG = routine.trajectory("MIDDLEtoG");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                MIDtoG.resetOdometry(),
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.POSITION_PREPARED_AUTO),
                MIDtoG.cmd()));

    MIDtoG.done()
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(
                WantedSuperState.AUTO_SCORE_L4_LEFT_AUTO));

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
  //             new WaitUntilCommand(() -> superstructureCommands.isWithinL4RaiseDistance())
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
