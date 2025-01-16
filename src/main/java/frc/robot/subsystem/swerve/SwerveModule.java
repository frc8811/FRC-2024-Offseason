package frc.robot.subsystem.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.Alert;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.genericsystem.*;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
  private static final LoggedTunableNumber driveKp =
      new LoggedTunableNumber(
          "Swerve/Module/DriveKp", SwerveConfig.getMk4iDriveTalonConfig().Slot0.kP);
  private static final LoggedTunableNumber driveKd =
      new LoggedTunableNumber(
          "Swerve/Module/DriveKd", SwerveConfig.getMk4iDriveTalonConfig().Slot0.kD);
  private static final LoggedTunableNumber driveKs =
      new LoggedTunableNumber(
          "Swerve/Module/DriveKs", SwerveConfig.getMk4iDriveTalonConfig().Slot0.kS);
  private static final LoggedTunableNumber steerKp =
      new LoggedTunableNumber(
          "Swerve/Module/SteerKp", SwerveConfig.getMk4iSteerTalonConfig().Slot0.kP);
  private static final LoggedTunableNumber steerKd =
      new LoggedTunableNumber(
          "Swerve/Module/SteerKd", SwerveConfig.getMk4iSteerTalonConfig().Slot0.kD);
  private static final LoggedTunableNumber steerKs =
      new LoggedTunableNumber(
          "Swerve/Module/SteerKs", SwerveConfig.getMk4iSteerTalonConfig().Slot0.kS);

  private final String name;

  private final AngularPositionIO steerIO;
  private final AngularVelocityIO driveIO;

  private final AngularIOInputsAutoLogged steerInputs = new AngularIOInputsAutoLogged();
  private final AngularIOInputsAutoLogged driveInputs = new AngularIOInputsAutoLogged();

  private final Alert driveKrakenOfflineAlert;
  private final Alert steerKrakenOfflineAlert;

  SwerveModule(String name, AngularPositionIO steerIO, AngularVelocityIO driveIO) {
    this.driveIO = driveIO;
    this.steerIO = steerIO;
    this.name = "Module " + name;

    driveKrakenOfflineAlert =
        new Alert(this.name + " drive motor offline!", Alert.AlertType.WARNING);
    steerKrakenOfflineAlert =
        new Alert(this.name + " steer motor offline!", Alert.AlertType.WARNING);
  }

  void updateInputs() {
    steerIO.updateInputs(steerInputs);
    driveIO.updateInputs(driveInputs);
    Logger.processInputs("Swerve/" + name + "/drive", driveInputs);
    Logger.processInputs("Swerve/" + name + "/steer", steerInputs);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> driveIO.setPdf(driveKp.get(), driveKd.get(), driveKs.get(), 0.),
        driveKp,
        driveKd,
        driveKs);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> steerIO.setPdf(steerKp.get(), steerKd.get(), steerKs.get(), 0.),
        steerKp,
        steerKd,
        steerKs);

    driveKrakenOfflineAlert.set(!driveInputs.connected);
    steerKrakenOfflineAlert.set(!steerInputs.connected);
  }

  void setState(SwerveModuleState state) {
    driveIO.setVelocity(state.speedMetersPerSecond / SwerveConfig.WHEEL_RADIUS_METER);
    steerIO.setPosition(state.angle.getRadians());
  }

  void setState(SwerveModuleState state, SwerveModuleState ff) {
    driveIO.setVelocity(
        state.speedMetersPerSecond / SwerveConfig.WHEEL_RADIUS_METER,
        ff.speedMetersPerSecond / SwerveConfig.DRIVE_FF_KT);
    steerIO.setPosition(state.angle.getRadians());
  }

  SwerveModuleState getState() {
    return new SwerveModuleState(
        driveInputs.velRadPerSec * SwerveConfig.WHEEL_RADIUS_METER,
        Rotation2d.fromRadians(steerInputs.positionRad));
  }

  double getDrivePositionRad() {
    return driveInputs.positionRad;
  }

  double getDriveVelRadPerSec() {
    return driveInputs.velRadPerSec;
  }

  double getSteerPositionRad() {
    return steerInputs.positionRad;
  }

  void setDriveCharacterizationCurrent(double currentAmp) {
    driveIO.setCurrent(currentAmp);
    steerIO.setPosition(0.);
  }

  void stop() {
    driveIO.stop();
    steerIO.stop();
  }
}
