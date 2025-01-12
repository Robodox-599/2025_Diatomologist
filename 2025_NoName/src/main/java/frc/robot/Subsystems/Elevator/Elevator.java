package frc.robot.Subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIO.ElevatorInputs inputs = new ElevatorIO.ElevatorInputs();
    
    public enum ElevatorSetpoints {
        LEVELONE, LEVELTWO, LEVELTHREE, LEVELFOUR
    }

    private ElevatorSetpoints currentSetpoint = ElevatorSetpoints.LEVELONE;

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    /* Clamps the elevators position so it can't go lower or higher than the lower and upper limits */
    public void setHeight(double position) {
        position = MathUtil.clamp(position, ElevatorConstants.elevatorLowerLimit, ElevatorConstants.elevatorUpperLimit);
        io.setHeight(position);
    }

    public void setHeightState(ElevatorSetpoints level) {
        currentSetpoint = level;
        double targetHeight = switch (level) {
            case LEVELONE -> ElevatorConstants.levelOneHeight;
            case LEVELTWO -> ElevatorConstants.levelTwoHeight;
            case LEVELTHREE -> ElevatorConstants.levelThreeHeight;
            case LEVELFOUR -> ElevatorConstants.levelFourHeight;
        };
        setHeight(targetHeight);
    }

    public boolean isAtTargetPosition() {
        return inputs.atSetpoint;
    }

    public ElevatorIO.ElevatorState getCurrentState() {
        return inputs.state;
    }

    public double getCurrentPosition() {
        return inputs.positionInches;
    }

    public double getCurrentVelocity() {
        return inputs.velocityInchesPerSec;
    }

    public ElevatorSetpoints getCurrentSetpoint() {
        return currentSetpoint;
    }

    /* Moves the elevator to one of the four setpoints */
    public Command moveToPosition(ElevatorSetpoints position) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                currentSetpoint = position;
                setHeightState(position);
             }),
            Commands.waitUntil(this::isAtTargetPosition)
        );
    }
    
    /* Moves the elevator back down to 0 inches */
    public Command homeElevator() {
        return Commands.sequence(
            Commands.runOnce(() -> {
                io.zeroEncoder();
                io.setHeight(ElevatorConstants.elevatorLowerLimit);
            }),
            Commands.waitUntil(() -> inputs.hallEffectTriggered),
            Commands.runOnce(() -> {
                io.setEncoder(ElevatorConstants.homePositionOffset);
                setHeight(ElevatorConstants.levelOneHeight);
            })
        );
    }
    
}