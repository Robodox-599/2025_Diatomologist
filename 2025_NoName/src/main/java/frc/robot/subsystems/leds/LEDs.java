package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.LEDsConstants.LEDAnim;

public class LEDs extends SubsystemBase {
    private final LEDsIO io;

    public LEDs(LEDsIO io) { //TODO: post integration, add other subsystems here so we can switch from running a command to using the periodic to grab subsystem states and update LEDs that way. 
        this.io = io;
    }

    public Command runStationIntake(){  
        return runOnce(
            ()-> io.enableStationIntake());
    }

    public Command runAlgaeIntake(){  
        return runOnce(
            ()-> io.enableAlgaeIntake());
    }

    public Command runNoState(){  
        return runOnce(
            ()-> io.enableNoState());
    }

    public Command runScored(){  
        return runOnce(
            ()-> io.enableScored());
    }

    public Command runClimb(){  
        return runOnce(
            ()-> io.enableClimb());
    }

    public Command runAutoAlign(){  
        return runOnce(
            ()-> io.enableAutoAlign());
    }

    public Command runReadyToScore(){  
        return runOnce(
            ()-> io.enableReadyToScore());
    }
    
}
