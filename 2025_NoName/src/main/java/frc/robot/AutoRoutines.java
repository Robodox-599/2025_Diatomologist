package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.commands.SuperstructureCommands;

public class AutoRoutines {
  private AutoFactory autoFactory;

  public AutoRoutines(AutoFactory autoFactory, SuperstructureCommands superstructureCommands) {
    this.autoFactory = autoFactory;
  }

  public AutoRoutine leftAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("leftAuto");

    // Load the routine's trajectories
    AutoTrajectory LEFTtoI = routine.trajectory("LEFTtoI");
    AutoTrajectory ItoHP = routine.trajectory("ItoHP");
    AutoTrajectory HPtoL = routine.trajectory("HPtoL");
    AutoTrajectory LtoHP = routine.trajectory("LtoHP");
    AutoTrajectory HPtoK = routine.trajectory("HPtoK");

    // When the routine begins, reset odometry and start the first trajectory
    routine.active().onTrue(Commands.sequence(LEFTtoI.resetOdometry(), LEFTtoI.cmd()));

    // When the trajectory is done, start the next trajectories
    // LEFTtoI.done().onTrue(ItoHP.cmd());
    // ItoHP.done().onTrue(HPtoL.cmd());
    // HPtoL.done().onTrue(LtoHP.cmd());
    // LtoHP.done().onTrue(HPtoK.cmd());

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

    // When the routine begins, reset odometry and start the first trajectory
    routine.active().onTrue(Commands.sequence(RIGHTtoF.resetOdometry(), RIGHTtoF.cmd()));

    // When the trajectory is done, start the next trajectory
    RIGHTtoF.done().onTrue(Commands.parallel(FtoHP.cmd()));
    FtoHP.done().onTrue(HPtoC.cmd());
    HPtoC.done().onTrue(CtoHP.cmd());
    CtoHP.done().onTrue(HPtoD.cmd());

    return routine;
  }

  public AutoRoutine middleAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("middleAuto");

    // Load the routine's trajectories
    AutoTrajectory MIDtoG = routine.trajectory("MIDtoG");

    // When the routine begins, reset odometry and start the first trajectory
    routine.active().onTrue(Commands.sequence(MIDtoG.resetOdometry(), MIDtoG.cmd()));

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
