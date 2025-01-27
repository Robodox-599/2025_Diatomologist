package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;


public class Elevator extends SubsystemBase {
    private final ElevatorIO io;

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs();
    }

    public boolean isAtTargetPosition() {
        return io.atSetpoint;
    }

    /* Moves the elevator to one of the states */
    public Command moveToState(ElevatorConstants.ElevatorStates state) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                io.setState(state);
             }),
            Commands.waitUntil(this::isAtTargetPosition)
        );
    }

    public Command move(double volt){
        return Commands.run(()->{
            io.setVoltage(volt);});
    }
    /*Homes elevator with limit switch, could be rewritten to home with current but hopefully nah*/

    public Command homeElevator() {
        return Commands.sequence(
            new InstantCommand(()->io.setVoltage(2)),
            new WaitUntilCommand(()-> io.limitSwitchValue),
            new InstantCommand(()-> io.setVoltage(0)),
            new InstantCommand(()-> io.zeroEncoder()))
        .onlyIf(
            ()-> !io.limitSwitchValue);
    }
    
}