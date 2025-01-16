package frc.robot.util.generic;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.util.Units;
import frc.robot.util.driver.CanId;

public class AngularVelocityIOKraken extends SuperKraken implements AngularVelocityIO {
  public AngularVelocityIOKraken(
      String name, CanId id, TalonFXConfiguration talonConfig, double homePositionDegree) {
    super(name, id, talonConfig, homePositionDegree);
  }

  @Override
  public void setVelocity(double velRadPerSec) {
    master.setControl(velocitySetter.withVelocity(Units.radiansToDegrees(velRadPerSec)));
    setSlavesFollow();
  }

  @Override
  public void setVelocity(double velRadPerSec, double feedforward) {
    master.setControl(
        motionMagicSetter
            .withVelocity(Units.radiansToDegrees(velRadPerSec))
            .withFeedForward(feedforward));
    setSlavesFollow();
  }

  @Override
  public void setVelocity(
      double velRadPerSec, double accelRadPerSecSq, double jerkRadPerSecCu, double feedforward) {
    master.setControl(
        motionMagicSetter
            .withVelocity(Units.radiansToDegrees(velRadPerSec))
            .withAcceleration(Units.radiansToDegrees(accelRadPerSecSq))
            .withJerk(Units.radiansToDegrees(jerkRadPerSecCu))
            .withFeedForward(feedforward));
    setSlavesFollow();
  }
}
