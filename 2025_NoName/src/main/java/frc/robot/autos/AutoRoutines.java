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
  private double waitTimeAtHP = 0.3;

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

    AutoTrajectory LEFTtoIBack = routine.trajectory("LEFTtoIBack");
    AutoTrajectory ItoHP = routine.trajectory("ItoHP");
    AutoTrajectory IBacktoHP = routine.trajectory("IBacktoHP");
    AutoTrajectory HPtoLBack = routine.trajectory("HPtoLBack");
    AutoTrajectory LtoHP = routine.trajectory("LtoHP");
    AutoTrajectory LBacktoHP = routine.trajectory("LBacktoHP");
    AutoTrajectory HPtoKBack = routine.trajectory("HPtoKBack");
    AutoTrajectory KtoHP = routine.trajectory("KtoHP");
    AutoTrajectory KBacktoHP = routine.trajectory("KBacktoHP");
    AutoTrajectory HPtoJBack = routine.trajectory("HPtoJBack");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                LEFTtoIBack.resetOdometry(),
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.POSITION_PREPARED_AUTO),
                LEFTtoIBack.cmd()));

    LEFTtoIBack.done()
        .onTrue(
            Commands.either(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.AUTO_SCORE_L4_LEFT_AUTO),
                IBacktoHP.cmd(),
                () -> !superstructureCommands.isCoralBranchScored()));

    LEFTtoIBack.recentlyDone()
        .and(() -> superstructureCommands.isCoralBranchScored())
        .and(IBacktoHP.inactive())
        .onTrue(
            Commands.sequence(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.INTAKING_CORAL_STATION),
                ItoHP.cmd()));

    routine
        .anyDone(ItoHP, IBacktoHP)
        .onTrue(Commands.sequence(new WaitCommand(waitTimeAtHP), HPtoLBack.cmd()));

    HPtoLBack.done()
        .onTrue(
            Commands.either(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.AUTO_SCORE_L4_RIGHT_AUTO),
                LBacktoHP.cmd(),
                () -> !superstructureCommands.isCoralBranchScored()));

    HPtoLBack.recentlyDone()
        .and(() -> superstructureCommands.isCoralBranchScored())
        .and(LBacktoHP.inactive())
        .onTrue(
            Commands.sequence(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.INTAKING_CORAL_STATION),
                LtoHP.cmd()));

    routine
        .anyDone(LtoHP, LBacktoHP)
        .onTrue(Commands.sequence(new WaitCommand(waitTimeAtHP), HPtoKBack.cmd()));

    HPtoKBack.done()
        .onTrue(
            Commands.either(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.AUTO_SCORE_L4_LEFT_AUTO),
                KBacktoHP.cmd(),
                () -> !superstructureCommands.isCoralBranchScored()));

    HPtoKBack.recentlyDone()
        .and(() -> superstructureCommands.isCoralBranchScored())
        .and(KBacktoHP.inactive())
        .onTrue(
            Commands.sequence(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.INTAKING_CORAL_STATION),
                KtoHP.cmd()));

    routine
        .anyDone(KtoHP, KBacktoHP)
        .onTrue(Commands.sequence(new WaitCommand(waitTimeAtHP), HPtoJBack.cmd()));

    HPtoJBack.recentlyDone()
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(
                WantedSuperState.AUTO_SCORE_L4_RIGHT_AUTO));

    return routine;
  }

  public AutoRoutine rightAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("rightAuto");

    AutoTrajectory RIGHTtoFBack = routine.trajectory("RIGHTtoFBack");
    AutoTrajectory FtoHP = routine.trajectory("FtoHP");
    AutoTrajectory FBacktoHP = routine.trajectory("FBacktoHP");
    AutoTrajectory HPtoCBack = routine.trajectory("HPtoCBack");
    AutoTrajectory CtoHP = routine.trajectory("CtoHP");
    AutoTrajectory CBacktoHP = routine.trajectory("CBacktoHP");
    AutoTrajectory HPtoDBack = routine.trajectory("HPtoDBack");
    AutoTrajectory DtoHP = routine.trajectory("DtoHP");
    AutoTrajectory DBacktoHP = routine.trajectory("DBacktoHP");
    AutoTrajectory HPtoEBack = routine.trajectory("HPtoEBack");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                RIGHTtoFBack.resetOdometry(),
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.POSITION_PREPARED_AUTO),
                RIGHTtoFBack.cmd()));

    RIGHTtoFBack.done()
        .onTrue(
            Commands.either(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.AUTO_SCORE_L4_RIGHT_AUTO),
                FBacktoHP.cmd(),
                () -> !superstructureCommands.isCoralBranchScored()));

    RIGHTtoFBack.recentlyDone()
        .and(() -> superstructureCommands.isCoralBranchScored())
        .and(FBacktoHP.inactive())
        .onTrue(
            Commands.sequence(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.INTAKING_CORAL_STATION),
                FtoHP.cmd()));

    routine
        .anyDone(FtoHP, FBacktoHP)
        .onTrue(Commands.sequence(new WaitCommand(waitTimeAtHP), HPtoCBack.cmd()));

    HPtoCBack.done()
        .onTrue(
            Commands.either(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.AUTO_SCORE_L4_LEFT_AUTO),
                CBacktoHP.cmd(),
                () -> !superstructureCommands.isCoralBranchScored()));

    HPtoCBack.recentlyDone()
        .and(() -> superstructureCommands.isCoralBranchScored())
        .and(CBacktoHP.inactive())
        .onTrue(
            Commands.sequence(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.INTAKING_CORAL_STATION),
                CtoHP.cmd()));

    routine
        .anyDone(CtoHP, CBacktoHP)
        .onTrue(Commands.sequence(new WaitCommand(waitTimeAtHP), HPtoDBack.cmd()));

    HPtoDBack.done()
        .onTrue(
            Commands.either(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.AUTO_SCORE_L4_RIGHT_AUTO),
                DBacktoHP.cmd(),
                () -> !superstructureCommands.isCoralBranchScored()));

    HPtoDBack.recentlyDone()
        .and(() -> superstructureCommands.isCoralBranchScored())
        .and(DBacktoHP.inactive())
        .onTrue(
            Commands.sequence(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.INTAKING_CORAL_STATION),
                DtoHP.cmd()));

    routine
        .anyDone(DtoHP, DBacktoHP)
        .onTrue(Commands.sequence(new WaitCommand(waitTimeAtHP), HPtoEBack.cmd()));

    HPtoEBack.recentlyDone()
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(
                WantedSuperState.AUTO_SCORE_L4_LEFT_AUTO));

    return routine;
  }

  //   public AutoRoutine rightAutoRoutine() {
  //     AutoRoutine routine = autoFactory.newRoutine("rightAuto");

  //     AutoTrajectory RIGHTtoF = routine.trajectory("RIGHTtoF");
  //     AutoTrajectory FtoHP = routine.trajectory("FtoHP");
  //     AutoTrajectory HPtoC = routine.trajectory("HPtoC");
  //     AutoTrajectory CtoHP = routine.trajectory("CtoHP");
  //     AutoTrajectory HPtoD = routine.trajectory("HPtoD");
  //     AutoTrajectory DtoHP = routine.trajectory("DtoHP");
  //     AutoTrajectory HPtoE = routine.trajectory("HPtoE");

  //     routine
  //         .active()
  //         .onTrue(
  //             Commands.sequence(
  //                 RIGHTtoF.resetOdometry(),
  //                 superstructureCommands.setWantedSuperStateCommand(
  //                     WantedSuperState.POSITION_PREPARED_AUTO),
  //                 RIGHTtoF.cmd()));

  //     RIGHTtoF.active()
  //         .and(
  //             () -> (drivetrain.isWithinL4RaiseDistance() &&
  // superstructureCommands.isCoralEnsured()))
  //         .onTrue(
  //             superstructureCommands.setWantedSuperStateInstantCommand(
  //                 WantedSuperState.POSITION_CORAL_L4));

  //     RIGHTtoF.recentlyDone()
  //         .onTrue(
  //
  // superstructureCommands.setWantedSuperStateCommand(WantedSuperState.AUTO_SCORE_L4_LEFT));

  //     RIGHTtoF.recentlyDone()
  //         .and(() -> superstructureCommands.isCoralBranchScored())
  //         .onTrue(
  //             Commands.parallel(
  //                 superstructureCommands.setWantedSuperStateCommand(
  //                     WantedSuperState.INTAKING_CORAL_STATION),
  //                 FtoHP.cmd()));

  //     FtoHP.done().onTrue(Commands.sequence(new WaitCommand(0.5), HPtoC.cmd()));

  //     HPtoC.active()
  //         .and(
  //             () -> (drivetrain.isWithinL4RaiseDistance() &&
  // superstructureCommands.isCoralEnsured()))
  //         .onTrue(
  //             superstructureCommands.setWantedSuperStateInstantCommand(
  //                 WantedSuperState.POSITION_CORAL_L4));

  //     HPtoC.recentlyDone()
  //         .onTrue(
  //             superstructureCommands.setWantedSuperStateCommand(
  //                 WantedSuperState.AUTO_SCORE_L4_RIGHT));

  //     HPtoC.recentlyDone()
  //         .and(() -> superstructureCommands.isCoralBranchScored())
  //         .onTrue(
  //             Commands.parallel(
  //                 superstructureCommands.setWantedSuperStateCommand(
  //                     WantedSuperState.INTAKING_CORAL_STATION),
  //                 CtoHP.cmd()));

  //     CtoHP.done().onTrue(Commands.sequence(new WaitCommand(0.5), HPtoD.cmd()));

  //     HPtoD.active()
  //         .and(
  //             () -> (drivetrain.isWithinL4RaiseDistance() &&
  // superstructureCommands.isCoralEnsured()))
  //         .onTrue(
  //             superstructureCommands.setWantedSuperStateInstantCommand(
  //                 WantedSuperState.POSITION_CORAL_L4));

  //     HPtoD.recentlyDone()
  //         .onTrue(
  //
  // superstructureCommands.setWantedSuperStateCommand(WantedSuperState.AUTO_SCORE_L4_LEFT));

  //     HPtoD.recentlyDone()
  //         .and(() -> superstructureCommands.isCoralBranchScored())
  //         .onTrue(
  //             Commands.parallel(
  //                 superstructureCommands.setWantedSuperStateCommand(
  //                     WantedSuperState.INTAKING_CORAL_STATION),
  //                 DtoHP.cmd()));

  //     DtoHP.done().onTrue(Commands.sequence(new WaitCommand(0.5), HPtoE.cmd()));

  //     HPtoE.active()
  //         .and(
  //             () -> (drivetrain.isWithinL4RaiseDistance() &&
  // superstructureCommands.isCoralEnsured()))
  //         .onTrue(
  //             superstructureCommands.setWantedSuperStateInstantCommand(
  //                 WantedSuperState.POSITION_CORAL_L4));

  //     HPtoE.recentlyDone()
  //         .onTrue(
  //             superstructureCommands.setWantedSuperStateCommand(
  //                 WantedSuperState.AUTO_SCORE_L4_RIGHT));

  //     return routine;
  //   }

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
            superstructureCommands.setWantedSuperStateCommand(WantedSuperState.POSITION_CORAL_L4));

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
