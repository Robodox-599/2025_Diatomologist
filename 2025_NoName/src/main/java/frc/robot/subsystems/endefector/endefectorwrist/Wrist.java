package frc.robot.subsystems.endefector.endefectorwrist;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SafetyChecker;
import frc.robot.subsystems.endefector.endefectorwrist.WristConstants.WristStates;
import frc.robot.util.SubsystemUtil;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final SafetyChecker safetyChecker;
  private WristConstants.WristStates internalState;

  public Wrist(WristIO io, SafetyChecker safetyChecker) {
    this.io = io;
    this.safetyChecker = safetyChecker;
    this.internalState = WristStates.STOW;
  }

  @Override
  public void periodic() {
    io.updateInputs();
    safetyChecker.setCurrentWristDegrees(io.currentPositionDegrees);
    // if (safetyChecker.isSafeWrist(SubsystemUtil.wristStateToSetpoint(internalState))) {
    //   io.setState(internalState);
    // }
  }

  public Command moveToState(WristConstants.WristStates state) {
    return this.run(
            () -> {
              this.internalState = state;
            })
        .until(() -> isAtTargetPosition(state));
  }

  public boolean isAtTargetPosition(WristConstants.WristStates state) {
    DogLog.log(
        "Wrist/IsAtTargetPosition",
        (Math.abs(io.currentPositionDegrees - SubsystemUtil.wristStateToSetpoint(state))
            < WristConstants.wristPositionTolerance));
    return (Math.abs(io.currentPositionDegrees - SubsystemUtil.wristStateToSetpoint(state))
        < WristConstants.wristPositionTolerance);
  }

  public WristConstants.WristStates getState() {
    return io.getCurrentState();
  }

  public Command stop() {
    return Commands.run(
        () -> {
          io.setVoltage(0);
        });
  }

  public WristIO getIO() {
    return io;
  }
}
