package frc.robot.subsystems.algaegroundintake.intakeRollers;

// import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollers extends SubsystemBase {
    private final IntakeRollersIO io;


    public IntakeRollers(IntakeRollersIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs();

    }

    //public double getVoltage() {
    //    return io.appliedVoltage; 
    //}

    public Command setVoltage(double voltage) {
        return Commands.run(
            () -> {
                io.setVoltage(voltage);
            });
    }

    public Command setVelocity(double velocity) {
        return Commands.run(
            () -> {
              io.setVelocity(velocity);
            }
            );
      }

      public Command stop() {
        return Commands.run(
            () -> {
              io.setVoltage(0);
            });
      }

      public void setBrake(boolean brake){
        io.setBrake(brake);
      }
      
      public IntakeRollersIO getIO(){
        return io;
      }


}
