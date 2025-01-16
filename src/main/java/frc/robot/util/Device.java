package frc.robot.util;

import frc.robot.util.driver.CanId;

public abstract class Device {
  protected String name;
  protected CanId id;

  public Device(String name, CanId id) {
    this.name = name;
    this.id = id;
  }
}
