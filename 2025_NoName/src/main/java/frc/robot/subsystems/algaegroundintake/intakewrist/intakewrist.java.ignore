package frc.robot.subsystems.algaegroundintake.intakeWrist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeWrist extends SubsystemBase {
  private final IntakeWristIO io;

  public IntakeWrist(IntakeWristIO io) {
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

  public Command moveToState(IntakeWristConstants.AlgaeStates state) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              io.setState(state);
            }),
        Commands.waitUntil(this::isAtTargetPosition));
  }

  public boolean isAtTargetPosition() {
    return io.atSetpoint;
  }

  public Command stop() {
    return Commands.run(
        () -> {
          io.setVoltage(0);
        });
  }

  public void setBrake(boolean brake) {
    io.setBrake(brake);
  }

  public IntakeWristIO getIO() {
    return io;
  }
}
