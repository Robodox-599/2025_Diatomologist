package frc.robot.subsystems.endefector.endefectorrollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.EndefectorRollerStates;

public class Rollers extends SubsystemBase {
  private final RollersIO io;

  public Rollers(RollersIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs();
  }

  public Command applyVoltage(double voltage) {
    return Commands.run(
        () -> {
          io.setVoltage(voltage);
        });
  }

  public Command setReverse(double voltage) {
    return Commands.run(
        () -> {
          io.setVoltage(-voltage);
        });
  }

  public Command moveToState(RollersConstants.EndefectorRollerStates state) {
    switch (state) {
      case INTAKE:
        return runRollersIntake();
      case SCORE:
        return runRollerScore();
      case ALGAEINTAKE:
        return runAlgaeIntake();
      case STOP:
        return Commands.runOnce(
            () -> {
              io.setState(state);
            });
      default:
        return Commands.runOnce(
            () -> {
              io.setState(state);
            });
    }
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

  public Command runAlgaeIntake() {
    if (io instanceof RollersIOSim) {
      return Commands.sequence(
          Commands.run(
                  () -> {
                    io.setState(EndefectorRollerStates.ALGAEINTAKE);
                  })
              .withTimeout(0.2),
          Commands.runOnce(() -> io.setState(EndefectorRollerStates.STOP)));
    } else {
      return Commands.sequence(
          Commands.run(
                  () -> {
                    io.setState(EndefectorRollerStates.ALGAEINTAKE);
                  })
              .until(() -> io.isAlgaeDetected));
    }
  }

  public Command runRollerScore() {
    if (io instanceof RollersIOSim) {
      return Commands.sequence(
          Commands.run(
                  () -> {
                    io.setState(EndefectorRollerStates.SCORE);
                  })
              .withTimeout(0.2),
          Commands.runOnce(() -> io.setState(EndefectorRollerStates.STOP)));
    } else {
      return Commands.sequence(
          Commands.run(
                  () -> {
                    io.setState(EndefectorRollerStates.SCORE);
                  })
              .until(io::isDetected),
          Commands.runOnce(
              () -> {
                io.setState(EndefectorRollerStates.STOP);
              }));
    }
  }

  public Command runRollersIntake() {
    if (io instanceof RollersIOSim) {
      return Commands.sequence(
          Commands.run(
                  () -> {
                    io.setState(EndefectorRollerStates.INTAKE);
                  })
              .withTimeout(0.2),
          Commands.runOnce(() -> io.setState(EndefectorRollerStates.STOP)));
    } else {
      return Commands.sequence(
          Commands.run(
                  () -> {
                    io.setState(EndefectorRollerStates.INTAKE);
                  })
              .until(io::isDetected),
          Commands.runOnce(
              () -> {
                io.setState(EndefectorRollerStates.STOP);
              }));
    }
  }

  public void setBrake(boolean brake) {
    io.setBrake(brake);
  }

  public RollersIO getIO() {
    return io;
  }

  public double getCoralDistance() {
    return io.getCoralDistance();
  }
}
