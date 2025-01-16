package frc.robot.util.genericsystem;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class LinearPositionIOSim extends AngularPositionIOSim implements AngularPositionIO {
  private static double meter2radian;

  public LinearPositionIOSim(
      DCMotor motor,
      double reduction,
      double moiJKgMeter2,
      double degree2meter,
      double offsetMeter) {
    super(motor, reduction, moiJKgMeter2, offsetMeter / degree2meter);
    meter2radian = 1. / Units.degreesToRadians(degree2meter);
  }

  @Override
  public void setPosition(double positionMeter) {
    pid.setConstraints(UNLIMITED);
    setVoltage(pid.calculate(sim.getAngularPositionRad(), positionMeter * meter2radian));
  }

  @Override
  public void setPosition(double positionMeter, double feedforward) {
    this.setPosition(positionMeter);
  }

  @Override
  public void setPosition(
      double positionMeter,
      double velMeterPerSec,
      double accelMeterPerSecSq,
      double jerkMeterPerSecCu,
      double feedforward) {
    pid.setConstraints(
        new TrapezoidProfile.Constraints(
            velMeterPerSec * meter2radian, accelMeterPerSecSq * meter2radian));
    setVoltage(pid.calculate(sim.getAngularPositionRad(), positionMeter * meter2radian));
  }
}
