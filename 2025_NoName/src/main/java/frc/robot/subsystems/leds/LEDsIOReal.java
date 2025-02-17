package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import dev.doglog.DogLog;

import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;

import frc.robot.subsystems.leds.LEDsConstants.LEDAnim;

public class LEDsIOReal extends LEDsIO {
   public final CANdle candleReal;
   private LEDAnim state = LEDAnim.NoState;

   public LEDsIOReal(){
    candleReal = new CANdle(LEDsConstants.canID, LEDsConstants.CANbus);
    CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = false;
        configAll.disableWhenLOS = true;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        candleReal.configAllSettings(configAll);
   }

   @Override
   public void updateInputs(){
        super.connected = true;
        super.anim = state;

        DogLog.log("LEDs/Connected", super.connected);
        DogLog.log("LEDs/Anim", super.anim);


   } 
//    @Override
//    public void updateAnim(LEDAnim anim){
//         state = anim;
//    }
   
   @Override
   public void enableStationIntake(){
    //candleReal.animate(new ColorFlowAnimation(255, 255, 70, 100, 0.85, LEDsConstants.LEDS_PER_ANIMATION, Direction.Forward, 0), 1);
    candleReal.animate(new LarsonAnimation(255, 255, 3), 1);
   }

   @Override
   public void enableAlgaeIntake(){
    //candleReal.animate(new ColorFlowAnimation(128, 0, 128, 0, 0.85, LEDsConstants.LEDS_PER_ANIMATION, Direction.Forward, 0), 1);
    candleReal.animate(new TwinkleAnimation(128, 0, 128), 1);
   }

   @Override
   public void enableNoState(){
    candleReal.setLEDs(0, 0, 0);
   }

   @Override
   public void enableScored(){
    candleReal.animate(new ColorFlowAnimation(12, 20, 70, 0, 0.85, LEDsConstants.LEDS_PER_ANIMATION, Direction.Forward, 0), 1);
   }

   @Override
   public void enableClimb(){
    candleReal.animate(new ColorFlowAnimation(0, 0, 255, 0, 0.85, LEDsConstants.LEDS_PER_ANIMATION, Direction.Forward, 0), 1);
   }

   @Override
   public void enableAutoAlign(){
    candleReal.animate(new ColorFlowAnimation(128, 20, 70, 0, 0.85, LEDsConstants.LEDS_PER_ANIMATION, Direction.Forward, 0), 1);
   }

   @Override
   public void enableReadyToScore(){
    candleReal.animate(new ColorFlowAnimation(128, 80, 70, 0, 0.7, LEDsConstants.LEDS_PER_ANIMATION, Direction.Forward, 0), 1);
   }
}