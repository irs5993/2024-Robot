// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
public final class Constants {
  public static class OperatorConstants {
    public static final int JOYSTICK_PORT = 0;
    public static final int kDriverControllerPort = 0;
  }

  public static class DriverPorts {
    public static final int CHASIS_LEFT = 0;
    public static final int CHASIS_RIGHT = 9;

    public static final int CONVEYOR = 1;
  }

  public static class CANIDS {
    public static final int SHOOTER_LEFT = 6;
    public static final int SHOOTER_RIGHT = 5;
    
    public static final int ARM_RIGHT = 3;
    public static final int ARM_LEFT = 4;
    public static final int ARM_ENCODER = 2;
  }
}
