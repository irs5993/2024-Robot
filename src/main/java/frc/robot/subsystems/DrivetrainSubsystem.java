// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriverPorts;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */

  private final PWMVictorSPX leftMotor;
  private final PWMVictorSPX rightMotor;

  private final DifferentialDrive driveBase;
  private final AHRS gyro;
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Yaw", getYaw());
    SmartDashboard.putNumber("Pitch", getPitch());
  }

  public DrivetrainSubsystem() {

    leftMotor = new PWMVictorSPX(DriverPorts.CHASIS_LEFT);
    rightMotor = new PWMVictorSPX(DriverPorts.CHASIS_RIGHT);
    rightMotor.setInverted(true);

    driveBase = new DifferentialDrive(leftMotor, rightMotor);
    gyro = new AHRS(SPI.Port.kMXP);
  }

  public void drive(double xSpeed, double zRotation) {
    driveBase.arcadeDrive(xSpeed, zRotation);
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
