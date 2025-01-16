package frc.robot.util.genericsystem;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class AngularVelocityIOSim extends SuperSim implements AngularIO {
  public AngularVelocityIOSim(DCMotor motor, double reduction, double moiJKgMeter2) {
    super(motor, reduction, moiJKgMeter2);
  }

  public void setVelocity(double velRadPerSec) {
    pid.setConstraints(UNLIMITED);
    setVoltage(pid.calculate(sim.getAngularVelocityRadPerSec(), velRadPerSec));
  }

  public void setVelocity(double velRadPerSec, double accelRadPerSecSq) {
    pid.setConstraints(
        new TrapezoidProfile.Constraints(Double.POSITIVE_INFINITY, accelRadPerSecSq));
    setVoltage(pid.calculate(sim.getAngularVelocityRadPerSec(), velRadPerSec));
  }
}
