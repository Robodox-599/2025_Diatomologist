package frc.robot.subsystems.endefector.coraldistance;
import static frc.robot.subsystems.endefector.coraldistance.CoralDistanceConstants.*;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

public class CoralDistanceIOReal extends CoralDistanceIO {
    private CANrange CANrange;
    private CANrangeConfiguration configs;

    public CoralDistanceIOReal(){
        CANrange CANrange = new CANrange(kCANrangeId, kCANrangeCANbus);
        CANrangeConfiguration configs = new CANrangeConfiguration();

        CANrange.getConfigurator().apply(configs);
    }

    @Override
    public boolean deviceDetected() {
        String deviceDetected = CANrange.getIsDetected().toString();
        boolean isDeviceDetected = false;

        if(deviceDetected == "true"){
            isDeviceDetected = true;
        } else {
            isDeviceDetected = false;
        }

        return isDeviceDetected;
       }

    @Override
    public double getDistance(){
        return CANrange.getDistance().getValueAsDouble();
    }

    @Override
    public double distancetoMove() {
        double coralDistance = CANrange.getDistance().getValueAsDouble()/39.3701;
        double needsToMove = 0.0;

        if(coralDistance >= coralLowerLimit && coralDistance <= coralUpperLimit) {
            needsToMove = 0;
        } else {
            if(coralDistance < coralLowerLimit){
                needsToMove = coralLowerLimit - coralDistance;
            }
            if(coralDistance > coralUpperLimit){
                needsToMove = coralDistance - coralUpperLimit;
            }
        }
        return needsToMove;
    }
}