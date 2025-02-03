package frc.robot.subsystems.leds;

import frc.robot.subsystems.leds.LEDsConstants.LEDAnim;

public abstract class LEDsIO {
    protected boolean connected = false;
    protected LEDAnim anim = LEDAnim.NoState;

    public void updateInputs(){}
    
    public void updateAnim(LEDAnim anim){}

    public LEDAnim getCurrentState(){return anim;}
    
}