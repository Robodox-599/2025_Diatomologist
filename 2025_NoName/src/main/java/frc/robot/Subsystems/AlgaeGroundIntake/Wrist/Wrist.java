package frc.robot.subsystems.algaegroundintake.wrist;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.algaegroundintake.wrist.WristIO.WristIOInputs;

public class Wrist extends SubsystemBase {
    private final WristIO io;

    public Wrist(WristIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(null);
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
                io.setVoltage(getPose());
            });
    }
}
