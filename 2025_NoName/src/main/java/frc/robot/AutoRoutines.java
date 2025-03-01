package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoRoutines {
  private AutoFactory autoFactory;

  public AutoRoutines(AutoFactory autoFactory) {
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

    // LEFTtoI.done().onTrue(Commands.sequence(RobotContainer.scoring(ElevatorStates.L4)));
    ItoHP.done().onTrue(new WaitCommand(1).andThen(HPtoL.cmd()));
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

  public AutoRoutine taxiAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("taxiAuto");

    // Load the routine's trajectories
    AutoTrajectory taxi = routine.trajectory("Taxi");

    // When the routine begins, reset odometry and start the first trajectory
    routine.active().onTrue(Commands.sequence(taxi.resetOdometry(), taxi.cmd()));

    return routine;
  }
}
