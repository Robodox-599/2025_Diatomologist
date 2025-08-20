package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
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

    Trigger withinCoralRaiseDistance =
        new Trigger(() -> superstructureCommands.isWithinCoralRaiseDistance());
    Trigger hasNoCoral = new Trigger(() -> superstructureCommands.hasCoral());

    routine
        .active()
        .onTrue(
            Commands.sequence(
                LEFTtoJ.resetOdometry(),
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.POSITION_PREPARED),
                LEFTtoJ.cmd()));

    LEFTtoJ.active()
        .and(withinCoralRaiseDistance)
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(WantedSuperState.POSITION_CORAL_L4));

    LEFTtoJ.done()
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(
                WantedSuperState.AUTO_SCORE_L4_RIGHT));

    LEFTtoJ.recentlyDone().and(hasNoCoral).onTrue(JtoHP.cmd());

    JtoHP.done().onTrue(HPtoL.cmd());

    HPtoL.active()
        .and(withinCoralRaiseDistance)
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(WantedSuperState.POSITION_CORAL_L4));

    HPtoL.done()
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(
                WantedSuperState.AUTO_SCORE_L4_RIGHT));

    HPtoL.recentlyDone().and(hasNoCoral).onTrue(LtoHP.cmd());

    LtoHP.done().onTrue(HPtoK.cmd());

    HPtoK.active()
        .and(withinCoralRaiseDistance)
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(WantedSuperState.POSITION_CORAL_L4));

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

    Trigger withinCoralRaiseDistance =
        new Trigger(() -> superstructureCommands.isWithinCoralRaiseDistance());
    Trigger hasNoCoral = new Trigger(() -> superstructureCommands.hasCoral());

    routine
        .active()
        .onTrue(
            Commands.sequence(
                RIGHTtoE.resetOdometry(),
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.POSITION_PREPARED),
                RIGHTtoE.cmd()));

    RIGHTtoE.active()
        .and(withinCoralRaiseDistance)
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(WantedSuperState.POSITION_CORAL_L4));

    RIGHTtoE.done()
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(WantedSuperState.AUTO_SCORE_L4_LEFT));

    RIGHTtoE.recentlyDone().and(hasNoCoral).onTrue(EtoHP.cmd());

    EtoHP.done().onTrue(HPtoC.cmd());

    HPtoC.active()
        .and(withinCoralRaiseDistance)
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(WantedSuperState.POSITION_CORAL_L4));

    HPtoC.done()
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(WantedSuperState.AUTO_SCORE_L4_LEFT));

    HPtoC.recentlyDone().and(hasNoCoral).onTrue(CtoHP.cmd());

    CtoHP.done().onTrue(HPtoD.cmd());

    HPtoD.active()
        .and(withinCoralRaiseDistance)
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(WantedSuperState.POSITION_CORAL_L4));

    HPtoD.done()
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(
                WantedSuperState.AUTO_SCORE_L4_RIGHT));

    return routine;
  }

  public AutoRoutine middleAutoAndGrabAlgaeRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("middleAutoAndGrabAlgae");

    AutoTrajectory MIDtoG = routine.trajectory("MIDDLEtoG");
    AutoTrajectory GtoGH = routine.trajectory("GtoGH");

    Trigger withinCoralRaiseDistance =
        new Trigger(() -> superstructureCommands.isWithinCoralRaiseDistance());
    Trigger hasNoCoral = new Trigger(() -> superstructureCommands.hasCoral());

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

    GtoGH.done()
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(
                superstructureCommands.returnAutoAlgaeIntakeState()));

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
}
