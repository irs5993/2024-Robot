// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static class Commands {

  }

  public static class Arm {
    // Position values for the arm movement limits
    public static final double MAX_POSITION = 0.207;
    public static final double MIN_POSITION = 0;

    // The position of the arm for retrieving the game piece from the human player
    public static final double HUMAN_POSITION = 0.141;

    // The default shooting position of the arm when the robot is touching it's
    // bumpers to the speaker
    public static final double DEFAULT_SHOOT_POSITION = 0.035;

    public static final double STAGE_SHOOT_POSITION = 0.074;

    // Determines when (or if) the arm position set by the PID loop is at it's
    // desired
    // location. Increasing this would make the arm respond less accurately to the
    // position updates
    public static final double CONTROLLER_TOLERANCE = 0.001;

    // Hard limit on the arm motor voltages. If the all the software limits
    // fail, this is the maximum voltage allowed on the motor controllers
    public static final double SAFETY_MAX_VOLTAGE = 0.75;

    // Offset for the arm position calculation. DO NOT change this unless you are
    // Arhan Burak Tüzün®™
    public static final double ANGLE_OFFSET = -15;

    public static final int SWITCH_PORT = 0;
  }

  public static class OperatorConstants {
    public static final int JOYSTICK_PORT = 0;
  }

  public static class Vision {
    public static final int PIPELINE_APRILTAG = 0;
  }

  public static class DriverPorts {
    public static final int CHASIS_LEAD_LEFT = 2;
    public static final int CHASIS_FOLLOWER_LEFT = 0;

    public static final int CHASIS_LEAD_RIGHT = 1;
    public static final int CHASIS_FOLLOWER_RIGHT = 3;

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
