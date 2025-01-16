package frc.robot.util.genericsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.util.Units;
import frc.robot.util.driver.CanId;

public class AngularVelocityIOKraken extends SuperKraken implements AngularIO {
  public AngularVelocityIOKraken(
      String name, CanId id, TalonFXConfiguration talonConfig, double homePositionDegree) {
    super(name, id, talonConfig, homePositionDegree);
  }

  public void setVelocity(double velRadPerSec) {
    master.setControl(velocitySetter.withVelocity(Units.radiansToDegrees(velRadPerSec)));
    setSlavesFollow();
  }

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
