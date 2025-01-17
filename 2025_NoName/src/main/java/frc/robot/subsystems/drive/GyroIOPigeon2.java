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
import frc.robot.subsystems.drive.constants.RealConstants;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 extends GyroIO {
  private final Pigeon2 pigeon = new Pigeon2(12, "LunaDriveCANivore");
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

  public GyroIOPigeon2() {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(RealConstants.ODOMETRY_FREQUENCY);
    yawVelocity.setUpdateFrequency(50.0);
    pigeon.optimizeBusUtilization();
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.getYaw());
  }

  @Override
  public void updateInputs() {
    super.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    super.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    super.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    super.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    super.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);

    DogLog.log("Drive/Gyro/Connected", super.connected);
    DogLog.log("Drive/Gyro/YawPosition", super.yawPosition);
    DogLog.log("Drive/Gyro/YawVelocity", super.yawVelocityRadPerSec);
    DogLog.log("Drive/Gyro/OdometryTimestamps", super.odometryYawTimestamps);
    DogLog.log("Drive/Gyro/OdometryPositions", super.odometryYawPositions);

    // Clear queues after logging
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
