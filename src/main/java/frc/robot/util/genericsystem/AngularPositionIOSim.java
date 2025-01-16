package frc.robot.util.genericsystem;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class AngularPositionIOSim extends SuperSim implements AngularPositionIO {
  private static double offsetRad;

  public AngularPositionIOSim(
      DCMotor motor, double reduction, double moiJKgMeter2, double offsetDegree) {
    super(motor, reduction, moiJKgMeter2);
    offsetRad = Units.degreesToRadians(offsetDegree);
  }

  @Override
  public void setPosition(double positionRad) {
    pid.setConstraints(UNLIMITED);
    setVoltage(pid.calculate(sim.getAngularPositionRad(), positionRad + offsetRad));
  }

  @Override
  public void setPosition(double positionRad, double feedforward) {
    setPosition(positionRad);
  }

  @Override
  public void setPosition(
      double positionRad,
      double velRadPerSec,
      double accelRadPerSecSq,
      double jerkRadPerSecCu,
      double feedforward) {
    pid.setConstraints(new TrapezoidProfile.Constraints(velRadPerSec, accelRadPerSecSq));
    setVoltage(pid.calculate(sim.getAngularPositionRad(), positionRad + offsetRad));
  }
}
