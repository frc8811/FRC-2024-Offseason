package frc.robot.subsystem.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Config;

public class SwerveConfig {
  static final String FL_MODULE_NAME = "FL";
  static final String BL_MODULE_NAME = "BL";
  static final String BR_MODULE_NAME = "BR";
  static final String FR_MODULE_NAME = "FR";

  static final double ODOMETRY_FREQUENCY_HZ = 250.0;

  static final boolean ENABLE_TRAJECTORY_FF = true;

  public static final double WHEELBASE_LENGTH_METER = 0.1877 * 2.0;
  public static final double WHEELBASE_WIDTH_METER = 0.21931 * 2.0;
  public static final double WHEELBASE_DIAGONAL_METER =
      Math.hypot(WHEELBASE_LENGTH_METER, WHEELBASE_WIDTH_METER);

  public static final double MAX_TRANSLATION_VEL_METER_PER_SEC = 5.4864;
  public static final double MAX_ANGULAR_VEL_RAD_PER_SEC =
      MAX_TRANSLATION_VEL_METER_PER_SEC / (WHEELBASE_DIAGONAL_METER / 2.0);

  public static final SwerveDriveKinematics SWERVE_KINEMATICS =
      new SwerveDriveKinematics(
          new Translation2d(+WHEELBASE_LENGTH_METER / 2.0, +WHEELBASE_WIDTH_METER / 2.0),
          new Translation2d(-WHEELBASE_LENGTH_METER / 2.0, +WHEELBASE_WIDTH_METER / 2.0),
          new Translation2d(-WHEELBASE_LENGTH_METER / 2.0, -WHEELBASE_WIDTH_METER / 2.0),
          new Translation2d(+WHEELBASE_LENGTH_METER / 2.0, -WHEELBASE_WIDTH_METER / 2.0));

  static final double WHEEL_RADIUS_METER = 0.0479;

  static final ModuleConfig FL_MODULE_CONFIG =
      new ModuleConfig(
          0.26904296875,
          getMk4iDriveTalonConfig(),
          getMk4iSteerTalonConfig(),
          getCancoderConfig(0.26904296875));
  static final ModuleConfig BL_MODULE_CONFIG =
      new ModuleConfig(
          0.12353515625,
          getMk4iDriveTalonConfig(),
          getMk4iSteerTalonConfig(),
          getCancoderConfig(0.12353515625));
  static final ModuleConfig BR_MODULE_CONFIG =
      new ModuleConfig(
          0.1640625,
          getMk4iDriveTalonConfig(),
          getMk4iSteerTalonConfig(),
          getCancoderConfig(0.1640625));
  static final ModuleConfig FR_MODULE_CONFIG =
      new ModuleConfig(
          0.35302734375,
          getMk4iDriveTalonConfig(),
          getMk4iSteerTalonConfig(),
          getCancoderConfig(0.35302734375));

  static final double DRIVE_REDUCTION = Mk4iReductions.L3.reduction;
  static final double STEER_REDUCTION = Mk4iReductions.TURN.reduction;
  static final double DRIVE_KG_METER_SQUARED = 0.025;
  static final double STEER_KG_METER_SQUARED = 0.004;

  static final double DRIVE_FF_KT =
      DCMotor.getKrakenX60Foc(1).withReduction(DRIVE_REDUCTION).KtNMPerAmp;

  record ModuleConfig(
      double offset,
      TalonFXConfiguration driveTalonConfig,
      TalonFXConfiguration steerTalonConfig,
      CANcoderConfiguration cancoderConfig) {}

  static Gains getDriveGains() {
    return switch (Config.MODE) {
      case REAL -> new Gains(35.0, 0.0, 0.0);
      case SIM, REPLAY -> new Gains(0.46, 0.0, 0.0);
    };
  }

  static Gains getSteerGains() {
    return switch (Config.MODE) {
      case REAL -> new Gains(700.0, 20.0, 0.0);
      case SIM, REPLAY -> new Gains(10.0, 0.0, 0.0);
    };
  }

  record Gains(double kp, double kd, double ks) {}

  static TalonFXConfiguration getMk4iDriveTalonConfig() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    var gains = getDriveGains();
    config.Slot0 = new Slot0Configs().withKP(gains.kp()).withKD(gains.kd()).withKS(gains.ks());

    config.TorqueCurrent.PeakForwardTorqueCurrent = 120.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -120.0;

    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 120.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Feedback.SensorToMechanismRatio = DRIVE_REDUCTION;

    return config;
  }

  static TalonFXConfiguration getMk4iSteerTalonConfig() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    var gains = getSteerGains();
    config.Slot0 = new Slot0Configs().withKP(gains.kp()).withKD(gains.kd()).withKS(gains.ks());
    config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;

    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 100.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Feedback.RotorToSensorRatio = STEER_REDUCTION;

    config.ClosedLoopGeneral.ContinuousWrap = true;

    return config;
  }

  private static CANcoderConfiguration getCancoderConfig(double magnetOffset) {
    var config = new CANcoderConfiguration();
    config.MagnetSensor.MagnetOffset = magnetOffset;

    return config;
  }

  private enum Mk4iReductions {
    L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)),
    L3((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)),
    TURN((50.0 / 14.0) * (60.0 / 10.0));

    final double reduction;

    Mk4iReductions(double reduction) {
      this.reduction = reduction;
    }
  }

  public static Pose3d FL_MODULE_ZEROED_POSE =
      new Pose3d(+WHEELBASE_LENGTH_METER / 2., +WHEELBASE_WIDTH_METER / 2., 0, new Rotation3d());
  public static Pose3d BL_MODULE_ZEROED_POSE =
      new Pose3d(-WHEELBASE_LENGTH_METER / 2., +WHEELBASE_WIDTH_METER / 2., 0, new Rotation3d());
  public static Pose3d BR_MODULE_ZEROED_POSE =
      new Pose3d(-WHEELBASE_LENGTH_METER / 2., -WHEELBASE_WIDTH_METER / 2., 0, new Rotation3d());
  public static Pose3d FR_MODULE_ZEROED_POSE =
      new Pose3d(+WHEELBASE_LENGTH_METER / 2., -WHEELBASE_WIDTH_METER / 2., 0, new Rotation3d());
}
