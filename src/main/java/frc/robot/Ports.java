package frc.robot;

import frc.robot.util.driver.CanId;

public class Ports {
  public static final class Can {
    public static final String RIO_BUS = "";
    public static final String CHASSIS_CANIVORE_BUS = "chassis";

    // IMU
    public static final CanId CHASSIS_PIGEON = new CanId(0, CHASSIS_CANIVORE_BUS);

    // Swerve
    public static final CanId FL_DRIVE_MOTOR = new CanId(0, CHASSIS_CANIVORE_BUS);
    public static final CanId FL_STEER_MOTOR = new CanId(1, CHASSIS_CANIVORE_BUS);
    public static final CanId FL_STEER_SENSOR = new CanId(2, CHASSIS_CANIVORE_BUS);

    public static final CanId BL_DRIVE_MOTOR = new CanId(3, CHASSIS_CANIVORE_BUS);
    public static final CanId BL_STEER_MOTOR = new CanId(4, CHASSIS_CANIVORE_BUS);
    public static final CanId BL_STEER_SENSOR = new CanId(5, CHASSIS_CANIVORE_BUS);

    public static final CanId BR_DRIVE_MOTOR = new CanId(6, CHASSIS_CANIVORE_BUS);
    public static final CanId BR_STEER_MOTOR = new CanId(7, CHASSIS_CANIVORE_BUS);
    public static final CanId BR_STEER_SENSOR = new CanId(8, CHASSIS_CANIVORE_BUS);

    public static final CanId FR_DRIVE_MOTOR = new CanId(9, CHASSIS_CANIVORE_BUS);
    public static final CanId FR_STEER_MOTOR = new CanId(10, CHASSIS_CANIVORE_BUS);
    public static final CanId FR_STEER_SENSOR = new CanId(11, CHASSIS_CANIVORE_BUS);
  }
}
