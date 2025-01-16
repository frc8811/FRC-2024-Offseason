package frc.robot.util.generic;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class AngularVelocityIOSim extends SuperSim implements AngularVelocityIO {
  public AngularVelocityIOSim(DCMotor motor, double reduction, double moiJKgMeter2) {
    super(motor, reduction, moiJKgMeter2);
  }

  @Override
  public void setVelocity(double velRadPerSec) {
    pid.setConstraints(UNLIMITED);
    setVoltage(pid.calculate(sim.getAngularVelocityRadPerSec(), velRadPerSec));
  }

  @Override
  public void setVelocity(double velRadPerSec, double feedforward) {
    setVelocity(velRadPerSec);
  }

  @Override
  public void setVelocity(
      double velRadPerSec, double accelRadPerSecSq, double jerkRadPerSecCu, double feedforward) {
    pid.setConstraints(
        new TrapezoidProfile.Constraints(Double.POSITIVE_INFINITY, accelRadPerSecSq));
    setVoltage(pid.calculate(sim.getAngularVelocityRadPerSec(), velRadPerSec));
  }
}
