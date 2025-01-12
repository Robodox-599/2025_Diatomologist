package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.leds.LEDsConstants.LEDAnim;

public interface LEDsIO {
    @AutoLog
    public static class LEDsIOInputs {
        public boolean connected = false;
        public LEDAnim anim = LEDAnim.NoState;
    } 

    public default void updateInputs(LEDsIOInputs inputs){}
    
    public default void updateAnim(LEDAnim anim){}
    
}