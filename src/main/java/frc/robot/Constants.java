// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static class Arm {
    public static final double MAX_POSITION = 0.2;
    public static final double MIN_POSITION = 0.004;

    // The position of the arm when the robot is touching it's
    // bumpers to the speaker
    public static final double DEFAULT_SHOOT_POSITION = 0.05;

    // Determines when (or if) the arm position set by the PID loop is at it's
    // desired
    // location. Increasing this would make the arm respond less accurately to the
    // position updates
    public static final double CONTROLLER_TOLERANCE = 0.001;

    // Hard limit on the arm motor voltages
    public static final double SAFETY_MAX_VOLTAGE = 0.75;
  }

  public static class OperatorConstants {
    public static final int JOYSTICK_PORT = 0;
  }

  public static class Vision {
    public static final int PIPELINE_APRILTAG = 0;
  }

  public static class DriverPorts {
    public static final int CHASIS_LEFT = 2;
    public static final int CHASIS_RIGHT = 1;

    public static final int CONVEYOR = 7;
  }

  public static class CANIDS {
    public static final int SHOOTER_TOP = 6;
    public static final int SHOOTER_BOTTOM = 5;

    public static final int ARM_RIGHT = 3;
    public static final int ARM_LEFT = 4;
    public static final int ARM_ENCODER = 10;
  }

}
