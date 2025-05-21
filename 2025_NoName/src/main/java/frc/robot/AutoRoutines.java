package frc.robot;

import choreo.auto.AutoFactory;
import frc.robot.subsystems.Superstructure;

public class AutoRoutines {
  private AutoFactory autoFactory;
  private Superstructure superstructureCommands;

  public AutoRoutines(AutoFactory autoFactory, Superstructure superstructureCommands) {
    this.autoFactory = autoFactory;
    this.superstructureCommands = superstructureCommands;
  }

  //   public AutoRoutine leftAutoRoutine() {
  //     AutoRoutine routine = autoFactory.newRoutine("leftAuto");

  //     AutoTrajectory LEFTtoJ = routine.trajectory("LEFTtoJ");
  //     AutoTrajectory JtoHP = routine.trajectory("JtoHP");
  //     AutoTrajectory HPtoL = routine.trajectory("HPtoL");
  //     AutoTrajectory LtoHP = routine.trajectory("LtoHP");
  //     AutoTrajectory HPtoK = routine.trajectory("HPtoK");

  //     routine
  //         .active()
  //         .onTrue(
  //             Commands.sequence(
  //                 LEFTtoJ.resetOdometry(),
  //                 // new WaitCommand(0.5),
  //                 superstructureCommands.autoIntakeFromStart(),
  //                 LEFTtoJ.cmd()));

  //     LEFTtoJ.done()
  //         .onTrue(
  //             Commands.sequence(
  //                 Commands.parallel(
  //                     superstructureCommands.moveToL4().withTimeout(1),
  //                     Commands.sequence(
  //                         new WaitCommand(0.1),
  //                         superstructureCommands.autoAlignToRight().withTimeout(5))),
  //                 superstructureCommands.scoreGamePieceWithoutIntaking(),
  //                 Commands.parallel(
  //                     Commands.sequence(
  //                         superstructureCommands.prepareToScore(),
  //                         superstructureCommands.coralStationIntake()),
  //                     JtoHP.cmd())));

  //     JtoHP.done().onTrue(Commands.sequence(new WaitCommand(0.75), HPtoL.cmd()));

  //     HPtoL.done()
  //         .onTrue(
  //             Commands.sequence(
  //                 Commands.parallel(
  //                     superstructureCommands.moveToL4().withTimeout(1),
  //                     Commands.sequence(
  //                         new WaitCommand(0.1),
  //                         superstructureCommands.autoAlignToRight().withTimeout(5))),
  //                 superstructureCommands.scoreGamePieceWithoutIntaking(),
  //                 Commands.parallel(
  //                     Commands.sequence(
  //                         superstructureCommands.prepareToScore(),
  //                         superstructureCommands.coralStationIntake()),
  //                     LtoHP.cmd())));

  //     LtoHP.done().onTrue(Commands.sequence(new WaitCommand(0.75), HPtoK.cmd()));

  //     HPtoK.done()
  //         .onTrue(
  //             Commands.sequence(
  //                 Commands.parallel(
  //                     superstructureCommands.moveToL4().withTimeout(1),
  //                     Commands.sequence(
  //                         new WaitCommand(0.1),
  //                         superstructureCommands.autoAlignToLeft().withTimeout(5))),
  //                 superstructureCommands.scoreGamePiece(),
  //                 superstructureCommands.prepareToScore()));

  //     return routine;
  //   }

  //   public AutoRoutine rightAutoRoutine() {
  //     AutoRoutine routine = autoFactory.newRoutine("rightAuto");

  //     AutoTrajectory RIGHTtoE = routine.trajectory("RIGHTtoE");
  //     AutoTrajectory EtoHP = routine.trajectory("EtoHP");
  //     AutoTrajectory HPtoC = routine.trajectory("HPtoC");
  //     AutoTrajectory CtoHP = routine.trajectory("CtoHP");
  //     AutoTrajectory HPtoD = routine.trajectory("HPtoD");

  //     routine
  //         .active()
  //         .onTrue(
  //             Commands.sequence(
  //                 RIGHTtoE.resetOdometry(),
  //                 // new WaitCommand(0.5),
  //                 superstructureCommands.autoIntakeFromStart(),
  //                 Commands.parallel(superstructureCommands.prepareToScore(), RIGHTtoE.cmd())));

  //     RIGHTtoE.done()
  //         .onTrue(
  //             Commands.sequence(
  //                 Commands.parallel(
  //                     superstructureCommands.moveToL4().withTimeout(0.8),
  //                     Commands.sequence(
  //                         new WaitCommand(0.1),
  //                         superstructureCommands.autoAlignToRight().withTimeout(5))),
  //                 superstructureCommands.scoreGamePieceWithoutIntaking(),
  //                 Commands.parallel(
  //                     Commands.sequence(
  //                         superstructureCommands.prepareToScore(),
  //                         superstructureCommands.coralStationIntake()),
  //                     EtoHP.cmd())));

  //     EtoHP.done().onTrue(Commands.sequence(new WaitCommand(0.75), HPtoC.cmd()));

  //     HPtoC.done()
  //         .onTrue(
  //             Commands.sequence(
  //                 Commands.parallel(
  //                     superstructureCommands.moveToL4().withTimeout(0.8),
  //                     Commands.sequence(
  //                         new WaitCommand(0.1),
  //                         superstructureCommands.autoAlignToRight().withTimeout(5))),
  //                 superstructureCommands.scoreGamePieceWithoutIntaking(),
  //                 Commands.parallel(
  //                     Commands.sequence(
  //                         superstructureCommands.prepareToScore(),
  //                         superstructureCommands.coralStationIntake()),
  //                     CtoHP.cmd())));

  //     CtoHP.done().onTrue(Commands.sequence(new WaitCommand(0.75), HPtoD.cmd()));

  //     HPtoD.done()
  //         .onTrue(
  //             Commands.sequence(
  //                 Commands.parallel(
  //                     superstructureCommands.moveToL4().withTimeout(0.8),
  //                     Commands.sequence(
  //                         new WaitCommand(0.1),
  //                         superstructureCommands.autoAlignToLeft().withTimeout(5))),
  //                 superstructureCommands.scoreGamePiece(),
  //                 superstructureCommands.prepareToScore()));

  //     return routine;
  //   }

  //   public AutoRoutine middleAutoRoutineWithAlgae() {
  //     AutoRoutine routine = autoFactory.newRoutine("middleAutoWithAlgae");

  //     AutoTrajectory MIDtoG = routine.trajectory("MIDtoG");
  //     AutoTrajectory GtoS4 = routine.trajectory("GtoS4");
  //     AutoTrajectory S4toNET = routine.trajectory("S4toNET");
  //     AutoTrajectory NETtoS5 = routine.trajectory("NETtoS5");
  //     AutoTrajectory S5toNET = routine.trajectory("S5toNET");

  //     routine
  //         .active()
  //         .onTrue(
  //             Commands.sequence(
  //                 MIDtoG.resetOdometry(),
  //                 // new WaitCommand(0.5),
  //                 superstructureCommands.autoIntakeFromStart(),
  //                 MIDtoG.cmd()));

  //     MIDtoG.done()
  //         .onTrue(
  //             Commands.sequence(
  //                 Commands.parallel(
  //                     superstructureCommands.moveToL4().withTimeout(0.8),
  //                     Commands.sequence(
  //                         new WaitCommand(0.1),
  //                         superstructureCommands.autoAlignToRight().withTimeout(5))),
  //                 superstructureCommands.scoreGamePieceWithoutIntaking(),
  //                 GtoS4.cmd()));

  //     GtoS4.done()
  //         .onTrue(
  //             Commands.sequence(
  //                 superstructureCommands.algaeIntake(ElevatorStates.ALGAEL2), S4toNET.cmd()));

  //     S4toNET.atTime("extendToNet").onTrue(superstructureCommands.extendToNet());

  //     S4toNET.done()
  //         .onTrue(
  //             Commands.sequence(
  //                 superstructureCommands.scoreGamePieceWithoutIntaking(),
  //                 Commands.parallel(
  //                     superstructureCommands.algaeIntake(ElevatorStates.ALGAEL3),
  // NETtoS5.cmd())));

  //     NETtoS5.done().onTrue(Commands.sequence(S5toNET.cmd()));

  //     S5toNET.atTime("extendToNet").onTrue(superstructureCommands.extendToNet());

  //     S5toNET.done().onTrue(superstructureCommands.scoreGamePiece());

  //     return routine;
  //   }

  //   public AutoRoutine middleAutoRoutine() {
  //     AutoRoutine routine = autoFactory.newRoutine("middleAuto");

  //     AutoTrajectory MIDtoG = routine.trajectory("MIDtoG");

  //     routine
  //         .active()
  //         .onTrue(
  //             Commands.sequence(
  //                 MIDtoG.resetOdometry(),
  //                 // new WaitCommand(0.5),
  //                 superstructureCommands.autoIntakeFromStart(),
  //                 MIDtoG.cmd()));

  //     MIDtoG.done()
  //         .onTrue(
  //             Commands.sequence(
  //                 superstructureCommands.moveToL4().withTimeout(5),
  //                 superstructureCommands.scoreCoralWithoutChecking(),
  //                 superstructureCommands.moveToL2()));

  //     return routine;
  //   }

  //   public AutoRoutine taxiAutoRoutine() {
  //     AutoRoutine routine = autoFactory.newRoutine("taxiAuto");

  //     AutoTrajectory MIDtoTaxi = routine.trajectory("MIDtoTaxi");

  //     routine.active().onTrue(Commands.sequence(MIDtoTaxi.resetOdometry(), MIDtoTaxi.cmd()));

  //     return routine;
  //   }

  //   public AutoRoutine testingAutoRoutine() {
  //     AutoRoutine routine = autoFactory.newRoutine("testingAuto");

  //     AutoTrajectory LEFTtoJ = routine.trajectory("LEFTtoJ");
  //     AutoTrajectory JtoHP = routine.trajectory("JtoHP");
  //     AutoTrajectory HPtoL = routine.trajectory("HPtoL");
  //     AutoTrajectory LtoHP = routine.trajectory("LtoHP");
  //     AutoTrajectory HPtoK = routine.trajectory("HPtoK");

  //     routine
  //         .active()
  //         .onTrue(
  //             Commands.sequence(
  //                 LEFTtoJ.resetOdometry(),
  //                 // new WaitCommand(0.5),
  //                 superstructureCommands.autoIntakeFromStart(),
  //                 LEFTtoJ.cmd()));

  //     LEFTtoJ.done()
  //         .onTrue(
  //             Commands.sequence(
  //                 Commands.parallel(
  //                     superstructureCommands.moveToL4().withTimeout(0.8),
  //                     Commands.sequence(
  //                         // new WaitCommand(0.1),
  //                         superstructureCommands.autoAlignToRight().withTimeout(3))),
  //                 superstructureCommands.scoreGamePieceWithoutIntaking(),
  //                 Commands.parallel(superstructureCommands.coralStationIntake(), JtoHP.cmd())));

  //     JtoHP.done().onTrue(HPtoL.cmd());

  //     HPtoL.done()
  //         .onTrue(
  //             Commands.sequence(
  //                 Commands.parallel(
  //                     superstructureCommands.moveToL4().withTimeout(0.8),
  //                     Commands.sequence(
  //                         new WaitCommand(0.1),
  //                         superstructureCommands.autoAlignToRight().withTimeout(3))),
  //                 superstructureCommands.scoreGamePieceWithoutIntaking(),
  //                 Commands.parallel(superstructureCommands.coralStationIntake(), LtoHP.cmd())));

  //     LtoHP.done().onTrue(HPtoK.cmd());

  //     HPtoK.done()
  //         .onTrue(
  //             Commands.sequence(
  //                 Commands.parallel(
  //                     superstructureCommands.moveToL4().withTimeout(0.8),
  //                     Commands.sequence(
  //                         new WaitCommand(0.1),
  //                         superstructureCommands.autoAlignToLeft().withTimeout(3))),
  //                 superstructureCommands.scoreGamePiece(),
  //                 superstructureCommands.prepareToScore()));

  //     return routine;
  //   }

  //   public AutoRoutine startTo15FeetAutoRoutine() {
  //     AutoRoutine routine = autoFactory.newRoutine("startTo15FeetAuto");

  //     AutoTrajectory STARTto15FT = routine.trajectory("STARTto15FT");

  //     routine.active().onTrue(Commands.sequence(STARTto15FT.resetOdometry(), STARTto15FT.cmd()));

  //     return routine;
  //   }
}
