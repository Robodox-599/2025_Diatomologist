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
    // read up here bums: https://choreo.autos/choreolib/auto-factory/
    public AutoRoutine simplePathAutoRoutine() {
        AutoRoutine routine = autoFactory.newRoutine("testAuto");

        // Load the routine's trajectories
        AutoTrajectory simplePath = routine.trajectory("SimplePath");

        // When the routine begins, reset odometry and start the first trajectory 
        routine.active().onTrue(
            Commands.sequence(
                simplePath.resetOdometry(),
                simplePath.cmd()
            )
        );
        return routine;
    }

}
