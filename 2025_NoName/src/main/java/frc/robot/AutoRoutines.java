package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoRoutines {
  private AutoFactory autoFactory;

  public AutoRoutines(AutoFactory autoFactory) {
    this.autoFactory = autoFactory;
  }

  public AutoRoutine leftAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("leftAuto");

    // Load the routine's trajectories
    AutoTrajectory LEFTtoI = routine.trajectory("LEFTtoI");

    // When the routine begins, reset odometry and start the first trajectory
    routine.active().onTrue(Commands.sequence(LEFTtoI.resetOdometry(), LEFTtoI.cmd()));

    return routine;
  }

  public AutoRoutine rightAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("rightAuto");

    // Load the routine's trajectories
    AutoTrajectory RIGHTtoF = routine.trajectory("RIGHTtoF");
    // AutoTrajectory FtoHP = routine.trajectory("FtoHP");
    // AutoTrajectory HPtoC = routine.trajectory("HPtoC");
    // When the routine begins, reset odometry and start the first trajectory
    routine.active().onTrue(Commands.sequence(RIGHTtoF.resetOdometry(), RIGHTtoF.cmd()));

    // When the trajectory is done, start the next trajectory
    // RIGHTtoF.done().onTrue(FtoHP.cmd());
    // FtoHP.done().onTrue(HPtoC.cmd());

    return routine;
  }

  public AutoRoutine turningTestAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("turningTestAuto");

    // Load the routine's trajectories
    AutoTrajectory turningTest = routine.trajectory("turningTest");

    // When the routine begins, reset odometry and start the first trajectory
    routine.active().onTrue(Commands.sequence(turningTest.resetOdometry(), turningTest.cmd()));

    return routine;
  }

  public AutoRoutine forwardBackTestAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("forwardBackTestAuto");

    // Load the routine's trajectories
    AutoTrajectory forwardBackTest = routine.trajectory("forwardBackTest");

    // When the routine begins, reset odometry and start the first trajectory
    routine
        .active()
        .onTrue(Commands.sequence(forwardBackTest.resetOdometry(), forwardBackTest.cmd()));

    return routine;
  }

  public AutoRoutine diamondTestAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("diamondTestAuto");

    // Load the routine's trajectories
    AutoTrajectory diamondTest = routine.trajectory("diamondTest");

    // When the routine begins, reset odometry and start the first trajectory
    routine.active().onTrue(Commands.sequence(diamondTest.resetOdometry(), diamondTest.cmd()));

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
