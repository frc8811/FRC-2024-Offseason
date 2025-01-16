package frc.robot.util.generic;

import org.littletonrobotics.junction.AutoLog;

public interface AngularIO {
  @AutoLog
  class AngularIOInputs {
    public boolean connected;

    public double positionRad;
    public double velRadPerSec;
    public double accelRadPerSecSq;
    public double temperatureCelsius;
    public double outputVoltageVolt;
    public double supplyCurrentAmp;
    public double statorCurrentAmp;
  }

  default void updateInputs(AngularIOInputs inputs) {}

  default void setPdf(double kp, double kd, double ks, double kg) {}

  default void setVoltage(double voltageVolt) {}

  default void setCurrent(double currentAmp) {}

  default void stop() {}
}
