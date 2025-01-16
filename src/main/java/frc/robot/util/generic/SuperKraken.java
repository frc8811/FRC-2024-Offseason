package frc.robot.util.generic;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.util.Device;
import frc.robot.util.driver.CanId;
import frc.robot.util.driver.PhoenixHelper;
import java.util.ArrayList;
import java.util.List;

public abstract class SuperKraken extends Device implements AngularIO {
  protected final TalonFX master;
  protected final List<TalonFX> slaves = new ArrayList<>();
  protected final TalonFXConfiguration masterConfig;

  protected final StatusSignal<Angle> position;
  protected final StatusSignal<AngularVelocity> velocity;
  protected final StatusSignal<AngularAcceleration> acceleration;
  protected final StatusSignal<Temperature> temperature;
  protected final StatusSignal<Voltage> outputVoltage;
  protected final StatusSignal<Current> supplyCurrent;
  protected final StatusSignal<Current> statorCurrent;

  protected final PositionTorqueCurrentFOC positionSetter;
  protected final VelocityTorqueCurrentFOC velocitySetter = new VelocityTorqueCurrentFOC(0.);
  protected final DynamicMotionMagicTorqueCurrentFOC motionMagicSetter;
  protected final VoltageOut voltageSetter = new VoltageOut(0.);
  protected final TorqueCurrentFOC currentSetter = new TorqueCurrentFOC(0.);
  protected final NeutralOut neutralSetter = new NeutralOut();
  protected final List<Follower> slaveSetters = new ArrayList<>();

  public SuperKraken(
      String name, CanId id, TalonFXConfiguration talonConfig, double homePositionDegree) {
    super(name, id);
    master = new TalonFX(id.id(), id.bus());
    masterConfig = talonConfig;

    String wrappedName = "[" + name + "]";
    PhoenixHelper.checkErrorAndRetry(
        wrappedName + " clear sticky fault", master::clearStickyFaults);
    PhoenixHelper.checkErrorAndRetry(
        wrappedName + " config", () -> master.getConfigurator().apply(masterConfig));
    PhoenixHelper.checkErrorAndRetry(
        wrappedName + " home position",
        () -> master.setPosition(Units.degreesToRotations(homePositionDegree)));

    positionSetter = new PositionTorqueCurrentFOC(Units.degreesToRotations(homePositionDegree));
    motionMagicSetter =
        new DynamicMotionMagicTorqueCurrentFOC(
            Units.degreesToRotations(homePositionDegree), 0., 0., 0.);

    position = master.getPosition();
    velocity = master.getVelocity();
    acceleration = master.getAcceleration();
    outputVoltage = master.getMotorVoltage();
    supplyCurrent = master.getSupplyCurrent();
    statorCurrent = master.getStatorCurrent();
    temperature = master.getDeviceTemp();

    PhoenixHelper.checkErrorAndRetry(
        wrappedName + " set signals update frequency",
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100.,
                position,
                velocity,
                acceleration,
                outputVoltage,
                supplyCurrent,
                statorCurrent,
                temperature));
    PhoenixHelper.checkErrorAndRetry(
        wrappedName + " optimize CAN utilization", master::optimizeBusUtilization);
  }

  public SuperKraken withFollower(CanId id, boolean isInvert) {
    var slave = new TalonFX(id.id(), id.bus());
    var slaveConfig = new TalonFXConfiguration();
    slaveConfig.MotorOutput.NeutralMode = masterConfig.MotorOutput.NeutralMode;
    var slaveSetter = new Follower(master.getDeviceID(), isInvert);

    var wrappedName = "[" + name + "Slave" + slaves.size() + "]";
    PhoenixHelper.checkErrorAndRetry(wrappedName + " clear sticky fault", slave::clearStickyFaults);
    PhoenixHelper.checkErrorAndRetry(
        wrappedName + " config", () -> slave.getConfigurator().apply(slaveConfig));
    PhoenixHelper.checkErrorAndRetry(wrappedName + " follow", () -> slave.setControl(slaveSetter));
    PhoenixHelper.checkErrorAndRetry(
        wrappedName + " optimize CAN utilization", slave::optimizeBusUtilization);

    slaves.add(slave);
    slaveSetters.add(slaveSetter);

    return this;
  }

  protected void setSlavesFollow() {
    for (int i = 0; i < slaves.size(); i++) {
      slaves.get(i).setControl(slaveSetters.get(i));
    }
  }

  @Override
  public void updateInputs(AngularIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                position,
                velocity,
                acceleration,
                outputVoltage,
                supplyCurrent,
                statorCurrent,
                temperature)
            .isOK();

    inputs.positionRad = position.getValueAsDouble();
    inputs.velRadPerSec = velocity.getValueAsDouble();
    inputs.accelRadPerSecSq = acceleration.getValueAsDouble();
    inputs.outputVoltageVolt = outputVoltage.getValueAsDouble();
    inputs.supplyCurrentAmp = supplyCurrent.getValueAsDouble();
    inputs.statorCurrentAmp = statorCurrent.getValueAsDouble();
    inputs.temperatureCelsius = temperature.getValueAsDouble();
  }

  @Override
  public void setPdf(double kp, double kd, double ks, double kg) {
    masterConfig.Slot0.kP = kp;
    masterConfig.Slot0.kD = kd;
    masterConfig.Slot0.kS = ks;
    masterConfig.Slot0.kG = kg;
    master.getConfigurator().apply(masterConfig, .01);
  }

  @Override
  public void setVoltage(double voltageVolt) {
    master.setControl(voltageSetter.withOutput(voltageVolt));
    setSlavesFollow();
  }

  @Override
  public void setCurrent(double currentAmp) {
    master.setControl(currentSetter.withOutput(currentAmp));
    setSlavesFollow();
  }

  @Override
  public void stop() {
    master.setControl(neutralSetter);
    setSlavesFollow();
  }
}
