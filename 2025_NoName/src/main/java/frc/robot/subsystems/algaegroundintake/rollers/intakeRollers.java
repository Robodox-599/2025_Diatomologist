package frc.robot.subsystems.algaegroundintake.rollers;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.algaegroundintake.wrist.intakeWrist;

public class intakeRollers extends SubsystemBase {
    private final RollersIO io;


    public intakeRollers(RollersIO io) {
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
      MechanismLigament2d m_rollers = 
            intakeWrist.m_wrist.append(new MechanismLigament2d("wrist", 0.5, 90, 6, new Color8Bit(Color.kPurple)));


}
