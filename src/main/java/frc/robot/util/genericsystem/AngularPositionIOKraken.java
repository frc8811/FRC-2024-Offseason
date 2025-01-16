package frc.robot.util.genericsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.util.Units;
import frc.robot.util.driver.CanId;

public class AngularPositionIOKraken extends SuperKraken implements AngularPositionIO {
  public AngularPositionIOKraken(
      String name, CanId id, TalonFXConfiguration talonConfig, double homePositionDegree) {
    super(name, id, talonConfig, homePositionDegree);
  }

  @Override
  public void setPosition(double positionRad) {
    master.setControl(positionSetter.withPosition(Units.radiansToDegrees(positionRad)));
    setSlavesFollow();
  }

  @Override
  public void setPosition(double positionRad, double feedforward) {
    master.setControl(
        motionMagicSetter
            .withPosition(Units.radiansToDegrees(positionRad))
            .withFeedForward(feedforward));
    setSlavesFollow();
  }

  @Override
  public void setPosition(
      double positionRad,
      double velRadPerSec,
      double accelRadPerSecSq,
      double jerkRadPerSecCu,
      double feedforward) {
    master.setControl(
        motionMagicSetter
            .withPosition(Units.radiansToDegrees(positionRad))
            .withVelocity(Units.radiansToDegrees(velRadPerSec))
            .withAcceleration(Units.radiansToDegrees(accelRadPerSecSq))
            .withJerk(Units.radiansToDegrees(jerkRadPerSecCu))
            .withFeedForward(feedforward));
    setSlavesFollow();
  }
}
