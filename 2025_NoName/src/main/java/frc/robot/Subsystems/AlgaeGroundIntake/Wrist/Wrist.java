package frc.robot.subsystems.algaegroundintake.wrist;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.algaegroundintake.wrist.WristIO.WristIOInputs;

public class Wrist extends SubsystemBase {
    private final WristIO io;
    private double setPoint = 0;
    private MechanismLigament2d wristMechanism = getArmMechanism();

    public Wrist(WristIO io) {
        this.io = io;
        SmartDashboard.putData(getName(), this);
    }

    @Override 
    public void periodic() {
        io.updateInputs(null);
        wristMechanism.setAngle(Units.radiansToDegrees(inputs.angleRads));
    }

    public Command goToPose(double pose) {
        return Command.run(
            () -> {
                io.goToPose(pose);
            });
    }

    public void goPID() {
        io.setSpeed(setPoint);
    }

    public void setPID(double setPoint) {
        this.setPoint = setPoint;
    }

    public boolean atSetpoint() {
        return Math.abs(io.getAngle() - setPoint) < WristConstants.intakeWristPositionTolerance 
        && Math.abs(getVelocity()) < WristConstants.intakeWristVelocityTolerance;
    }

    private double getVelocity() {
            return inputs.angleVelocityRadsPerSec;
    }

    public MechanismLigament2d getArmMechanism() {
        return new MechanismLigament2d("IntakeWrist", 0.4, 0, 5, new Color8Bit(Color.kAqua));
    }

    public Command PIDCommand(DoubleSupplier setPointSupplier) {
        return new FunctionalCommand(
            () -> setPID(setPointSupplier.getAsDouble()),
            () -> {
                setPID(setPointSupplier.getAsDouble());
                goPID();
            },
            (stop) -> stop(),
            this::atSetpoint,
            this);

    }

    public Command stop() {
        return new FunctionalCommand(
            () -> {}, () -> io.setVoltage(0), (stop) -> io.stop(), () -> false, this);
    }
}
