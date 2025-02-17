package frc.robot.subsystems.leds;

import dev.doglog.DogLog;
import frc.robot.subsystems.leds.LEDsConstants.LEDAnim;

public class LEDsIOSim extends LEDsIO {
    private LEDAnim state = LEDAnim.NoState;
    public LEDsIOSim(){
    }
 
    @Override
    public void updateInputs(){
         super.connected = true;
         super.anim = state;

         DogLog.log("LEDs/Connected", super.connected);
         DogLog.log("LEDs/Anim", super.anim);
    } 
//     @Override
//     public void updateAnim(LEDAnim anim){
//          state = anim;
//     }
}