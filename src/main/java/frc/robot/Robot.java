package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.util.driver.CanId;
import frc.robot.util.generic.AngularPositionIOKraken;
import org.littletonrobotics.junction.LoggedRobot;

public class Robot extends LoggedRobot {
  AngularPositionIOKraken kraken =
      new AngularPositionIOKraken("a", new CanId(1, "rio"), new TalonFXConfiguration(), 0);

  public Robot() {}

  public void robotPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
