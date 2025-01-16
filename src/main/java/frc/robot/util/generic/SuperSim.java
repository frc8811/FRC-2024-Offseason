package frc.robot.util.generic;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Config;

public abstract class SuperSim implements AngularIO {
  static final TrapezoidProfile.Constraints UNLIMITED =
      new TrapezoidProfile.Constraints(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

  protected final DCMotorSim sim;
  protected final ProfiledPIDController pid =
      new ProfiledPIDController(0., 0., 0., UNLIMITED, Config.LOOP_PERIOD_SEC);
  protected final SlewRateLimiter voltageLimiter = new SlewRateLimiter(2.5);

  private double appliedVoltageVolt = 0.0;

  public SuperSim(DCMotor motor, double reduction, double moiJKgMeter2) {
    sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, moiJKgMeter2, reduction), motor);
  }

  @Override
  public void updateInputs(AngularIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      stop();
      setVoltage(voltageLimiter.calculate(appliedVoltageVolt));
    } else {
      voltageLimiter.reset(appliedVoltageVolt);
    }

    sim.update(Config.LOOP_PERIOD_SEC);

    inputs.connected = true;
    inputs.positionRad = sim.getAngularPositionRad();
    inputs.velRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.accelRadPerSecSq = sim.getAngularAccelerationRadPerSecSq();
    inputs.temperatureCelsius = 45;
    inputs.supplyCurrentAmp = Math.abs(sim.getCurrentDrawAmps());
    inputs.statorCurrentAmp = Math.abs(sim.getCurrentDrawAmps());
    inputs.outputVoltageVolt = appliedVoltageVolt;
  }

  @Override
  public void setPdf(double kp, double kd, double ks, double kg) {
    pid.setPID(kp, 0., kd);
  }

  @Override
  public void setVoltage(double voltageVolt) {
    appliedVoltageVolt = MathUtil.clamp(voltageVolt, -12., 12.);
    sim.setInputVoltage(appliedVoltageVolt);
  }

  @Override
  public void setCurrent(double currentAmp) {
    // TODO
  }

  @Override
  public void stop() {
    setVoltage(0.);
  }
}
