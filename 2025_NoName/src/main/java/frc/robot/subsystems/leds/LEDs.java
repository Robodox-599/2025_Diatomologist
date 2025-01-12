package frc.robot.subsystems.leds;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.LEDsConstants.LEDAnim;

public class LEDs extends SubsystemBase {
    private final LEDsIO io;
    private final LEDsIOInputsAutoLogged inputs = new LEDsIOInputsAutoLogged();

    public LEDs(LEDsIO io) { //TODO: post integration, add other subsystems here so we can switch from running a command to using the periodic to grab subsystem states and update LEDs that way. 
        this.io = io;
    }
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("LEDs", inputs);
    }

    public Command runAnim(LEDAnim anim){
        return runOnce(
            ()-> io.updateAnim(anim));
    }

}
