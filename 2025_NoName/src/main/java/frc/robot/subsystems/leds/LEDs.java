package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.LEDsConstants.LEDAnim;

public class LEDs extends SubsystemBase {
    private final LEDsIO io;

    public LEDs(LEDsIO io) { //TODO: post integration, add other subsystems here so we can switch from running a command to using the periodic to grab subsystem states and update LEDs that way. 
        this.io = io;
    }

    public Command runAnim(LEDAnim anim){
        
        return runOnce(
            ()-> io.updateAnim(anim)).andThen(Commands.print(anim.toString()));
    }

}
