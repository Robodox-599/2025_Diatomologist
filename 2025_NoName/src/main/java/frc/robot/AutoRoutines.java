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

  public AutoRoutine rightAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("rightAuto");

    // Load the routine's trajectories
    AutoTrajectory RIGHTtoF = routine.trajectory("RIGHTtoF");
    AutoTrajectory FtoHP = routine.trajectory("FtoHP");

    // When the routine begins, reset odometry and start the first trajectory
    routine.active().onTrue(Commands.sequence(RIGHTtoF.resetOdometry(), RIGHTtoF.cmd()));

    // When the trajectory is done, start the next trajectory
    RIGHTtoF.done().onTrue(FtoHP.cmd());

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
