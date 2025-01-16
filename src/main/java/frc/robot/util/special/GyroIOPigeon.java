package frc.robot.util.special;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.Device;
import frc.robot.util.driver.CanId;
import frc.robot.util.driver.PhoenixHelper;
import lombok.Getter;
import org.jetbrains.annotations.NotNull;

public class GyroIOPigeon extends Device implements GyroIO {
  private final Pigeon2 pigeon;
  @Getter private final StatusSignal<Angle> yaw;
  private final StatusSignal<AngularVelocity> yawVelocity;

  GyroIOPigeon(String name, CanId id) {
    super(name, id);
    pigeon = new Pigeon2(id.id(), id.bus());
    yaw = pigeon.getYaw();
    yawVelocity = pigeon.getAngularVelocityZWorld();

    String wrappedName = "[" + name + "]";
    PhoenixHelper.checkErrorAndRetry(
        wrappedName + " Gyro config",
        () -> pigeon.getConfigurator().apply(new Pigeon2Configuration()));
    PhoenixHelper.checkErrorAndRetry(
        wrappedName + " Gyro zero", () -> pigeon.getConfigurator().setYaw(0.));
    PhoenixHelper.checkErrorAndRetry(
        wrappedName + " Gyro set signal update update frequency",
        () -> yawVelocity.setUpdateFrequency(100.));
    PhoenixHelper.checkErrorAndRetry(
        wrappedName + " Gyro optimize CAN utilization", pigeon::optimizeBusUtilization);
  }

  @Override
  public void updateInputs(@NotNull GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).isOK();

    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
  }
}
