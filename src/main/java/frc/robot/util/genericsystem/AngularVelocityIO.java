package frc.robot.util.genericsystem;

public interface AngularVelocityIO extends AngularIO {
  default void setVelocity(double velRadPerSec) {}

  default void setVelocity(double velRadPerSec, double feedforward) {}

  default void setVelocity(
      double velRadPerSec, double accelRadPerSecSq, double jerkRadPerSecCu, double feedforward) {}
}
