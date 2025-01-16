package frc.robot.util.genericsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.util.Units;
import frc.robot.util.driver.CanId;

public class AngularPositionIOKraken extends SuperKraken implements AngularIO {
  public AngularPositionIOKraken(
      String name, CanId id, TalonFXConfiguration talonConfig, double homePositionDegree) {
    super(name, id, talonConfig, homePositionDegree);
  }

  public void setPosition(double positionRad) {
    master.setControl(positionSetter.withPosition(Units.radiansToDegrees(positionRad)));
    setSlavesFollow();
  }

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
