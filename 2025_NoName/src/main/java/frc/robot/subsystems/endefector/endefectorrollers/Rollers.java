package frc.robot.subsystems.endefector.endefectorrollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.EndefectorRollerStates;

public class Rollers extends SubsystemBase {
  private final RollersIO io;
  private RollersConstants.EndefectorRollerStates internalState;

  public Rollers(RollersIO io) {
    this.io = io;
    this.internalState = EndefectorRollerStates.STOP;
  }

  @Override
  public void periodic() {
    io.updateInputs();
    io.setState(internalState);
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
                  this.internalState = EndefectorRollerStates.ALGAEINTAKE;
                })
            .until(() -> io.isAlgaeDetected),
        Commands.runOnce(() -> this.internalState = EndefectorRollerStates.HOLDALGAE));
  }

  public Command runScoreAlgae() {
    return Commands.sequence(
        Commands.run(
                () -> {
                  this.internalState = EndefectorRollerStates.SCOREALGAE;
                })
            .until(() -> !io.isAlgaeDetected),
        Commands.runOnce(
            () -> {
              this.internalState = EndefectorRollerStates.STOP;
            }));
  }

  public Command runCoralStationIntake() {
    return Commands.sequence(
        Commands.run(
                () -> {
                  this.internalState = EndefectorRollerStates.CORALSTATIONINTAKE;
                })
            .until(() -> io.isCoralDetected),
        Commands.runOnce(
            () -> {
              this.internalState = EndefectorRollerStates.ADJUSTCORALAFTERSTATIONINTAKE;
            }));
  }

  public Command runScoreCoral() {
    return Commands.sequence(
        Commands.run(
                () -> {
                  this.internalState = EndefectorRollerStates.SCORECORAL;
                })
            .until(() -> !io.isCoralDetected),
        Commands.runOnce(
            () -> {
              this.internalState = EndefectorRollerStates.STOP;
            }));
  }

  public Command ejectGamePiece() {
    return Commands.run(
        () -> {
          this.internalState = EndefectorRollerStates.EJECT;
        });
  }

  public boolean isCoralDetected() {
    return io.isCoralDetected;
  }

  public boolean isAlgaeDetected() {
    return io.isAlgaeDetected;
  }
}
