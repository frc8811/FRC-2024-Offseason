package frc.robot.util.genericsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.util.Units;
import frc.robot.util.driver.CanId;

public class LinearPositionIOKraken extends AngularPositionIOKraken implements AngularIO {
  private static double meter2degree;

  public LinearPositionIOKraken(
      String name,
      CanId id,
      TalonFXConfiguration talonConfig,
      double degree2meter,
      double homePositionMeter) {
    super(name, id, talonConfig, homePositionMeter / degree2meter);
    meter2degree = 1 / degree2meter;
  }

  @Override
  public void setPosition(double positionMeter) {
    master.setControl(
        positionSetter.withPosition(Units.radiansToDegrees(positionMeter * meter2degree)));
    setSlavesFollow();
  }

  @Override
  public void setPosition(
      double positionMeter,
      double velMeterPerSec,
      double accelMeterPerSecSq,
      double jerkMeterPerSecCu,
      double feedforward) {
    master.setControl(
        motionMagicSetter
            .withPosition(positionMeter * meter2degree)
            .withVelocity(velMeterPerSec * meter2degree)
            .withAcceleration(accelMeterPerSecSq * meter2degree)
            .withJerk(jerkMeterPerSecCu * meter2degree)
            .withFeedForward(feedforward));
    setSlavesFollow();
  }
}
