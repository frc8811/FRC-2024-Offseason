package frc.robot.subsystem.swerve;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.Ports;
import frc.robot.util.generic.*;

public class Swerve extends SubsystemBase {
  private final SwerveModule[] modules = new SwerveModule[4];

  public Swerve() {
    switch (Config.MODE) {
      case REAL -> {
        var flSteerIO =
            new AngularPositionIOKraken(
                SwerveConfig.FL_MODULE_NAME + "_STEER",
                Ports.Can.FL_STEER_MOTOR,
                SwerveConfig.FL_MODULE_CONFIG.steerTalonConfig(),
                Units.rotationsToDegrees(SwerveConfig.FL_MODULE_CONFIG.offset()));
        var flDriveIO =
            new AngularVelocityIOKraken(
                SwerveConfig.FL_MODULE_NAME + "_DRIVE",
                Ports.Can.FL_DRIVE_MOTOR,
                SwerveConfig.FL_MODULE_CONFIG.driveTalonConfig(),
                Double.NaN);
        var blSteerIO =
            new AngularPositionIOKraken(
                SwerveConfig.BL_MODULE_NAME + "_STEER",
                Ports.Can.BL_STEER_MOTOR,
                SwerveConfig.BL_MODULE_CONFIG.steerTalonConfig(),
                Units.rotationsToDegrees(SwerveConfig.BL_MODULE_CONFIG.offset()));
        var blDriveIO =
            new AngularVelocityIOKraken(
                SwerveConfig.BL_MODULE_NAME + "_DRIVE",
                Ports.Can.BL_DRIVE_MOTOR,
                SwerveConfig.BL_MODULE_CONFIG.driveTalonConfig(),
                Double.NaN);
        var brSteerIO =
            new AngularPositionIOKraken(
                SwerveConfig.BR_MODULE_NAME + "_STEER",
                Ports.Can.BR_STEER_MOTOR,
                SwerveConfig.BR_MODULE_CONFIG.steerTalonConfig(),
                Units.rotationsToDegrees(SwerveConfig.BR_MODULE_CONFIG.offset()));
        var brDriveIO =
            new AngularVelocityIOKraken(
                SwerveConfig.BR_MODULE_NAME + "_DRIVE",
                Ports.Can.BR_DRIVE_MOTOR,
                SwerveConfig.BR_MODULE_CONFIG.driveTalonConfig(),
                Double.NaN);
        var frSteerIO =
            new AngularPositionIOKraken(
                SwerveConfig.FR_MODULE_NAME + "_STEER",
                Ports.Can.FR_STEER_MOTOR,
                SwerveConfig.FR_MODULE_CONFIG.steerTalonConfig(),
                Units.rotationsToDegrees(SwerveConfig.FR_MODULE_CONFIG.offset()));
        var frDriveIO =
            new AngularVelocityIOKraken(
                SwerveConfig.FR_MODULE_NAME + "_DRIVE",
                Ports.Can.FR_DRIVE_MOTOR,
                SwerveConfig.FR_MODULE_CONFIG.driveTalonConfig(),
                Double.NaN);

        modules[0] = new SwerveModule(SwerveConfig.FL_MODULE_NAME, flSteerIO, flDriveIO);
        modules[1] = new SwerveModule(SwerveConfig.BL_MODULE_NAME, blSteerIO, blDriveIO);
        modules[2] = new SwerveModule(SwerveConfig.BR_MODULE_NAME, brSteerIO, brDriveIO);
        modules[3] = new SwerveModule(SwerveConfig.FR_MODULE_NAME, frSteerIO, frDriveIO);

        // TODO
      }
      case SIM -> {
        var flSteerIO =
            new AngularPositionIOSim(
                DCMotor.getKrakenX60Foc(1),
                SwerveConfig.STEER_REDUCTION,
                SwerveConfig.STEER_KG_METER_SQUARED,
                SwerveConfig.FL_MODULE_CONFIG.offset());
        var flDriveIO =
            new AngularVelocityIOSim(
                DCMotor.getKrakenX60Foc(1),
                SwerveConfig.DRIVE_REDUCTION,
                SwerveConfig.DRIVE_KG_METER_SQUARED);
        var blSteerIO =
            new AngularPositionIOSim(
                DCMotor.getKrakenX60Foc(1),
                SwerveConfig.STEER_REDUCTION,
                SwerveConfig.STEER_KG_METER_SQUARED,
                SwerveConfig.BL_MODULE_CONFIG.offset());
        var blDriveIO =
            new AngularVelocityIOSim(
                DCMotor.getKrakenX60Foc(1),
                SwerveConfig.DRIVE_REDUCTION,
                SwerveConfig.DRIVE_KG_METER_SQUARED);
        var brSteerIO =
            new AngularPositionIOSim(
                DCMotor.getKrakenX60Foc(1),
                SwerveConfig.STEER_REDUCTION,
                SwerveConfig.STEER_KG_METER_SQUARED,
                SwerveConfig.BR_MODULE_CONFIG.offset());
        var brDriveIO =
            new AngularVelocityIOSim(
                DCMotor.getKrakenX60Foc(1),
                SwerveConfig.DRIVE_REDUCTION,
                SwerveConfig.DRIVE_KG_METER_SQUARED);
        var frSteerIO =
            new AngularPositionIOSim(
                DCMotor.getKrakenX60Foc(1),
                SwerveConfig.STEER_REDUCTION,
                SwerveConfig.STEER_KG_METER_SQUARED,
                SwerveConfig.FR_MODULE_CONFIG.offset());
        var frDriveIO =
            new AngularVelocityIOSim(
                DCMotor.getKrakenX60Foc(1),
                SwerveConfig.DRIVE_REDUCTION,
                SwerveConfig.DRIVE_KG_METER_SQUARED);

        modules[0] = new SwerveModule(SwerveConfig.FL_MODULE_NAME, flSteerIO, flDriveIO);
        modules[1] = new SwerveModule(SwerveConfig.BL_MODULE_NAME, blSteerIO, blDriveIO);
        modules[2] = new SwerveModule(SwerveConfig.BR_MODULE_NAME, brSteerIO, brDriveIO);
        modules[3] = new SwerveModule(SwerveConfig.FR_MODULE_NAME, frSteerIO, frDriveIO);

        // TODO
      }
      default -> {
        var flSteerIO = new AngularPositionIO() {};
      }
    }
  }

  private void updateInputs() {
    // TODO

    for (var module : modules) {
      module.updateInputs();
    }
  }

  @Override
  public void periodic() {
    updateInputs();
  }
}
