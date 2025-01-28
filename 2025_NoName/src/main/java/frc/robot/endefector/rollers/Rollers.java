package frc.robot.endefector.rollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase{
    private final RollersIO io;

    public Rollers(RollersIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs();
    }

    public Command setVoltage(double voltage) {
        return Commands.run(
            () -> {
                io.setVoltage(voltage);
            });
    }

    public Command stop() {
        return Commands.run(
            () -> {
                io.setVoltage(0);
            });
    }

    public Command setVelocity(double velocity) {
        return Commands.run(
            () -> {
                io.setVelocity(velocity);
            });
    }

    public void setBrake(boolean brake) {
        io.setBrake(brake);
    }
}
