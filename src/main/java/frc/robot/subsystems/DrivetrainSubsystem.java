// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriverPorts;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Yeni bir DrivetrainSubsystem yaratır. */

  private final PWMVictorSPX leftLeadMotor;
  private final PWMVictorSPX leftFollowerMotor;

  private final PWMVictorSPX rightLeadMotor;
  private final PWMVictorSPX rightFollowerMotor;

  private final MotorControllerGroup leftGroup;
  private final MotorControllerGroup rightGroup;

  private final DifferentialDrive driveBase;
  private final AHRS gyro;

  public DrivetrainSubsystem() {
    leftLeadMotor = new PWMVictorSPX(DriverPorts.CHASIS_LEAD_LEFT);
    leftFollowerMotor = new PWMVictorSPX(DriverPorts.CHASIS_FOLLOWER_LEFT);

    rightLeadMotor = new PWMVictorSPX(DriverPorts.CHASIS_LEAD_RIGHT);
    rightFollowerMotor = new PWMVictorSPX(DriverPorts.CHASIS_FOLLOWER_RIGHT);

    rightLeadMotor.setInverted(true);
    rightFollowerMotor.setInverted(true);

    leftGroup = new MotorControllerGroup(leftLeadMotor, leftFollowerMotor);
    rightGroup = new MotorControllerGroup(rightLeadMotor, rightFollowerMotor);

    driveBase = new DifferentialDrive(leftGroup, rightGroup);
    gyro = new AHRS(SPI.Port.kMXP);
    gyro.reset();
  }

  // Bu yöntem her zamanlayıcı çalıştırmasında bir kez çağrılacak
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Yaw", getYaw());
    SmartDashboard.putNumber("Pitch", getPitch());
  }

  public void drive(double xSpeed, double zRotation) {
    driveBase.arcadeDrive(xSpeed, -zRotation);
  }

  public void resetGyro() {
    gyro.reset();
  }

  public double getYaw() {
    return gyro.getYaw();
  }

  public double getPitch() {
    return gyro.getPitch();
  }

  public void stop() {
    driveBase.stopMotor();
  }
}
