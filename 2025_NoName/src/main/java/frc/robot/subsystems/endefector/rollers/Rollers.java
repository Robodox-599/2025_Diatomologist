import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import static frc.robot.subsystems.endefector.rollers.RollersConstants.*;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj.Timer;

public class Rollers extends SubsystemBase{
    private final RollersIO io;
    private Timer CANRangeTimer = new Timer();
    private  CANrange CANrange;
    

    public Rollers(RollersIO io) {
        this.io = io;
        CANRangeTimer.start();
    }

    public void periodic() {
        io.updateInputs();
        if (rangeDeviceDetected()) {
            CANRangeTimer.restart();
          }
    }
    
    public boolean rangeDeviceDetected(){
        double rangeSignal = 0.0;
        rangeSignal = CANrange.getDistance().getValueAsDouble();
    
        if (rangeSignal >= rangeTolerance) {
            return true;
        } else {
            return false;
        }
    }

    public Command applyVoltage(double voltage){
        
    }

    public Command runIndexerBeamBreak() {
    return Commands.sequence(
        new InstantCommand(() -> io.setVoltage(-0.4), this),
        new WaitUntilCommand(() -> (CANRangeTimer.get() >= 0.1)),
        new InstantCommand(() -> io.setVoltage(0), this));
  }
 }

