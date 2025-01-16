package frc.robot.genericsystem;

import org.littletonrobotics.junction.AutoLog;

public interface GenericAngularIO {
  @AutoLog
  class GenericAngularIOInputs {
    public boolean connected;

    public double positionRad;
    public double velRadPerSec;
    public double accelRadPerSecSq;
    public double outputVoltageVolt;
    public double supplyCurrentAmp;
    public double statorCurrentAmp;
  }

  default void updateInputs(GenericAngularIOInputs inputs) {}

  default void setPosition(double positionRad) {}

  default void setVelocity(double velRadPerSec) {}

  default void setFeedforward(double feedforward) {}

  default void setConstraints(double velRadPerSec, double accelRadPerSecSq) {}

  default void setPdf(double kp, double kd, double kf) {}

  default void setVoltage(double voltageVolts) {}

  default void setCurrent(double currentAmp) {}

  default void stop() {}
}
