package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import dev.doglog.DogLog;

import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import frc.robot.subsystems.leds.LEDsConstants.LEDAnim;

public class LEDsIOReal extends LEDsIO {
   public final CANdle candle;
   private LEDAnim state = LEDAnim.NoState;
   public LEDsIOReal(){
    candle = new CANdle(LEDsConstants.canID, LEDsConstants.CANbus);
    CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = false;
        configAll.disableWhenLOS = true;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        candle.configAllSettings(configAll);
   }

   @Override
   public void updateInputs(){
        super.connected = true;
        super.anim = state;

        DogLog.log("LEDs/Connected", super.connected);
        DogLog.log("LEDs/Anim", super.anim);


   } 
   @Override
   public void updateAnim(LEDAnim anim){
        state = anim;        
        // candle.setLEDs(255,0,0);

        // switch(anim){
            // default:
        //     case StationIntake:
                // channel = 0;
                // candle.setLEDs(255,0,0);
        //         break;
        //     case Scored:
        //         channel = 1;
            if(anim == LEDAnim.ReadyToScore)
                candle.animate(new ColorFlowAnimation(128, 20, 70, 0, 0.7, LEDsConstants.LEDS_PER_ANIMATION, Direction.Forward, 0), 1);
            else {
                candle.setLEDs(0, 0, 0);
            }
        //         break;
        //     case AutoAlign:
        //         channel = 2;
        //         candle.animate(new RainbowAnimation(1, 0.7, LEDsConstants.LEDS_PER_ANIMATION, false, channel * LEDsConstants.LEDS_PER_ANIMATION + 8), channel);
        //         break;
        //     case Climb:
        //         channel = 3;
        //         candle.animate(new StrobeAnimation(240, 10, 180, 0, 0.01, LEDsConstants.LEDS_PER_ANIMATION, channel * LEDsConstants.LEDS_PER_ANIMATION + 8), channel);
        //         break;
        //     case AlgaeIntake:
        //         channel = 3;
        //         candle.animate(new StrobeAnimation(240, 10, 180, 0, 0.01, LEDsConstants.LEDS_PER_ANIMATION, channel * LEDsConstants.LEDS_PER_ANIMATION + 8), channel);
        //         break;
        //     case ReadyToScore:
        //         channel = 3;
        //         candle.animate(new StrobeAnimation(240, 10, 180, 0, 0.01, LEDsConstants.LEDS_PER_ANIMATION, channel * LEDsConstants.LEDS_PER_ANIMATION + 8), channel);
        //         break;
        // }
   }
}