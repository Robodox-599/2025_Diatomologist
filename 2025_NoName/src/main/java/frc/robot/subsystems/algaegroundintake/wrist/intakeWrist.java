package frc.robot.subsystems.algaegroundintake.wrist;


import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeWrist extends SubsystemBase {
    private final WristIO io;

    public IntakeWrist(WristIO io) {
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

    public Command setVoltage(double voltage) {
        return Commands.run(
            () -> {
                io.setVoltage(voltage);
            });
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

    static Mechanism2d mech = new Mechanism2d(1, 2);
        static MechanismRoot2d   root = mech.getRoot("wrist", 1, 2);

        public static MechanismLigament2d m_wrist = root.append(new MechanismLigament2d("wrist", 6, 90));
}
