package frc.robot.util.genericsystem;

public interface AngularPositionIO extends AngularIO {
  default void setPosition(double positionRad) {}

  default void setPosition(double positionRad, double feedforward) {}

  default void setPosition(
      double positionRad,
      double velRadPerSec,
      double accelRadPerSecSq,
      double jerkRadPerSecCu,
      double feedforward) {}
}
