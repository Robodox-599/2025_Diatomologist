package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import frc.robot.subsystems.leds.LEDsConstants.LEDAnim;

public abstract class LEDsIO {
    protected boolean connected = false;
    protected LEDAnim anim = LEDAnim.NoState;

    public void updateInputs(){}
    
    public void updateAnim(LEDAnim anim){}
}