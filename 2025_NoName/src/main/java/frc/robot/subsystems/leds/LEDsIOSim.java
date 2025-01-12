package frc.robot.subsystems.leds;

import frc.robot.subsystems.leds.LEDsConstants.LEDAnim;

public class LEDsIOSim implements LEDsIO {
    private LEDAnim state = LEDAnim.NoState;
    public LEDsIOSim(){
    }
 
    @Override
    public void updateInputs(LEDsIOInputs inputs ){
         inputs.connected = true;
         inputs.anim = state;
    } 
    @Override
    public void updateAnim(LEDAnim anim){
         state = anim;
    }
}