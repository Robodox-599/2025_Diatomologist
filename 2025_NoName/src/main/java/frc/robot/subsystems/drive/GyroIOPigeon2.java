package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.drive.constants.generated.TunerConstants;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon =
      new Pigeon2(
          TunerConstants.DrivetrainConstants.Pigeon2Id,
          TunerConstants.DrivetrainConstants.CANBusName);
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

  public GyroIOPigeon2() {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
    yawVelocity.setUpdateFrequency(50.0);
    pigeon.optimizeBusUtilization();
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.getYaw());
  }

  @Override
  public boolean getConnected() {
    boolean connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    DogLog.log("Drive/Gyro/Connected", connected);
    return connected;
  }

  @Override
  public Rotation2d getYawPosition() {
    Rotation2d position = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    DogLog.log("Drive/Gyro/YawPosition", position.getDegrees());
    return position;
  }

  @Override
  public double getYawVelocityRadPerSec() {
    double velocity = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    DogLog.log("Drive/Gyro/YawVelocity", velocity);
    return velocity;
  }

  @Override
  public double[] getOdometryYawTimestamps() {
    double[] timestamps = yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    DogLog.log("Drive/Gyro/OdometryTimestamps", timestamps);
    return timestamps;
  }

  @Override
  public Rotation2d[] getOdometryYawPositions() {
    Rotation2d[] positions = yawPositionQueue.stream()
        .map((Double value) -> Rotation2d.fromDegrees(value))
        .toArray(Rotation2d[]::new);
        DogLog.log("Drive/Gyro/OdometryPositions", positions);
    return positions;
  }

  @Override
  public void updateInputs() { 
    DogLog.log("Drive/Gyro/Connected", getConnected());
    DogLog.log("Drive/Gyro/YawPosition", getYawPosition().getDegrees());
    DogLog.log("Drive/Gyro/YawVelocity", getYawVelocityRadPerSec());
    DogLog.log("Drive/Gyro/OdometryTimestamps", getOdometryYawTimestamps());
    DogLog.log("Drive/Gyro/OdometryPositions", getOdometryYawPositions());

    // Clear queues after logging
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }

}
