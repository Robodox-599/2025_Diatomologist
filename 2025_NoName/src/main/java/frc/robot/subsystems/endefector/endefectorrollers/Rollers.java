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

  @Override
  public void periodic() {
    io.updateInputs();
  }

  public Command stop() {
    return Commands.run(
        () -> {
          io.stop();
        });
  }

  public Command setVelocity(double velocity) {
    return Commands.run(
        () -> {
          io.setVelocity(velocity);
        });
  }

  public Command runAlgaeIntake() {
    return Commands.sequence(
        Commands.run(
                () -> {
                  io.setState(EndefectorRollerStates.ALGAEINTAKE);
                })
            .until(() -> io.isAlgaeDetected()),
        Commands.runOnce(() -> io.setState(EndefectorRollerStates.HOLDALGAE)));
  }

  public Command runScoreAlgae() {
    return Commands.sequence(
        Commands.run(
                () -> {
                  io.setState(EndefectorRollerStates.SCOREALGAE);
                })
            .until(() -> !io.isAlgaeDetected()),
        Commands.runOnce(
            () -> {
              io.setState(EndefectorRollerStates.STOP);
            }));
  }

  public Command runCoralStationIntake() {
    return Commands.sequence(
        Commands.run(
                () -> {
                  io.setState(EndefectorRollerStates.CORALSTATIONINTAKE);
                })
            .until(() -> io.isCoralDetected()),
        Commands.runOnce(
            () -> {
              io.setState(EndefectorRollerStates.ADJUSTCORALAFTERSTATIONINTAKE);
            }));
  }

  public Command runScoreCoral() {
    return Commands.sequence(
        Commands.run(
                () -> {
                  io.setState(EndefectorRollerStates.SCORECORAL);
                })
            .until(() -> !io.isCoralDetected()),
        Commands.runOnce(
            () -> {
              io.setState(EndefectorRollerStates.STOP);
            }));
  }

  public Command ejectGamePiece() {
    return Commands.run(
        () -> {
          io.setState(EndefectorRollerStates.EJECT);
        });
  }

  public boolean isCoralDetected() {
    return io.isCoralDetected();
  }

  public boolean isAlgaeDetected() {
    return io.isAlgaeDetected();
  }

  public RollersIO getIO() {
    return io;
  }

  // public void setBrake(boolean brake) {
  //   io.setBrake(brake);
  // }

  //   public double getCoralDistance() {
  //     return io.getCoralDistance();
  //   }

  // public Command applyVoltage(double voltage) {
  //   return Commands.run(
  //       () -> {
  //         io.setVoltage(voltage);
  //       });
  // }
}
