package frc.robot.endefector.claw;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase{
    private final ClawIO io;

    public Claw(ClawIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs();
    }

    public Command goToPose(double pose) {
        return Commands.run(
            () -> {
                io.goToPose(pose);
            });
    }

    public double getPose() {
        return io.getPose();
    }

    public void setBrake(boolean brake) {
        io.setBrake(brake);
    }

    public Command stop() {
        return Commands.run(
            () -> {
                io.setVoltage(0);
            });
    }
}
