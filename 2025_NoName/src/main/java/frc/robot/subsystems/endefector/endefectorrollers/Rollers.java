package frc.robot.subsystems.endefector.endefectorrollers;

// import static frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.EndefectorRollerStates;

// import frc.robot.subsystems.endefector.coraldistance.CoralDistance;
// import frc.robot.subsystems.endefector.coraldistance.CoralDistanceIOReal;

public class Rollers extends SubsystemBase {
  private final RollersIO io;
  private Timer CANRangeTimer = new Timer();

  // private CoralDistance canrange;

  public Rollers(RollersIO io) {
    this.io = io;
    // canrange = new CoralDistance(new CoralDistanceIOReal());
    // CANRangeTimer.start();
  }

  public void periodic() {
    io.updateInputs();
    // canrange.deviceDetected();
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
          io.setVoltage(voltage * -1);
        });
  }

  public Command moveToState(RollersConstants.EndefectorRollerStates state) {
    switch (state) {
      case INTAKE:
        return runRollersIntake();
      case SCORE:
        return runRollerScore();
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
    // io.setState(state);
    // return Commands.runOnce(
    //         () -> {

    //         })
    //     .withTimeout(0.6);
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

  public Command runRollersIntake() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              io.setState(EndefectorRollerStates.INTAKE);
            }),
        new WaitUntilCommand(() -> (CANRangeTimer.get() >= 0.1)),
        Commands.runOnce(
            () -> {
              io.setState(EndefectorRollerStates.STOP);
            }));
  }

  public Command runRollerScore() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              io.setState(EndefectorRollerStates.SCORE);
            }),
        new WaitUntilCommand(() -> (CANRangeTimer.get() >= 0.1)),
        Commands.runOnce(
            () -> {
              io.setState(EndefectorRollerStates.STOP);
            }));
  }

  //     public boolean rangeDeviceDetected(){
  //     double rangeSignal = 0.0;
  //     rangeSignal = CANrange.getDistance().getValueAsDouble();

  //     if (rangeSignal >= rangeTolerance) {
  //         return true;
  //     } else {
  //         return false;
  //     }
  // }

  public RollersIO getIO() {
    return io;
  }
}
