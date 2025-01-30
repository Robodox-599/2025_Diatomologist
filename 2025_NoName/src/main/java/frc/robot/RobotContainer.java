package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

//Endefector stuff
import frc.robot.subsystems.endefector.rollers.Rollers;
import frc.robot.subsystems.endefector.rollers.RollersIOSim;
import frc.robot.subsystems.endefector.rollers.RollersIOTalonFX;
import frc.robot.subsystems.endefector.wrist.Wrist;
import frc.robot.subsystems.endefector.wrist.WristIOSim;
import frc.robot.subsystems.endefector.wrist.WristIOTalonFX;
import frc.robot.subsystems.subsystemvisualizer.subsystemvisualizer;
//Climb stuff
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOTalonFX;

//Intake stuff
//import frc.robot.subsystems.algaegroundintake.rollers.Rollers;
// import frc.robot.subsystems.algaegroundintake.rollers.RollersIOSim;
// import frc.robot.subsystems.algaegroundintake.rollers.RollersIOTalonFX;
// import frc.robot.subsystems.algaegroundintake.wrist.Wrist;
// import frc.robot.subsystems.algaegroundintake.wrist.WristIOSim;
// import frc.robot.subsystems.algaegroundintake.wrist.WristIOTalonFX;

public class RobotContainer {
  private final CommandXboxController controller = 
    new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
  
  private subsystemvisualizer subsystemvisualizer;
  //Endefector
  private Rollers endefectorRollers;
  private Wrist endefectorWrist;
  //Climb
  private Climb climb;

    public RobotContainer() {
      switch (Constants.currentMode) {
        case REAL:
          subsystemvisualizer = new subsystemvisualizer();
          endefectorRollers = new Rollers(new RollersIOTalonFX());
          endefectorWrist = new Wrist(new WristIOTalonFX());
          climb = new Climb(new ClimbIOTalonFX());
            break;
        case SIM:
          subsystemvisualizer = new subsystemvisualizer();
          endefectorRollers = new Rollers(new RollersIOSim());
          endefectorWrist = new Wrist(new WristIOSim());
          climb = new Climb(new ClimbIOSim());
          break;
      }
    
          DogLog.setOptions(
          new DogLogOptions().withCaptureDs(true).withCaptureNt(true).withNtPublish(true));

          configureBindings();
    }
    private void configureBindings() {
      //climb
      controller.a().whileTrue(climb.move(2));
      //Endefector
      controller.x().whileTrue(endefectorWrist.goToPose(2));
      controller.b().whileTrue(endefectorRollers.setVelocity(2));
    }
      
      
    public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
    }
}