package frc.robot.util.generic;

public interface AngularVelocityIO extends AngularIO {
  default void setVelocity(double velRadPerSec) {}

  default void setVelocity(double velRadPerSec, double feedforward) {}

  default void setVelocity(
      double velRadPerSec, double accelRadPerSecSq, double jerkRadPerSecCu, double feedforward) {}
}
