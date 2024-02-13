// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {

  private final CANSparkMax leftMotor;
  private final CANSparkMax rightMotor;

  public ShooterSubsystem() {
    leftMotor = new CANSparkMax(CANIDS.SHOOTER_LEFT, MotorType.kBrushless);
    rightMotor = new CANSparkMax(CANIDS.SHOOTER_RIGHT, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setShooterSpeed(double speed) {
    setLeftMotorSpeed(speed);
    setRightMotorSpeed(speed);
  }

  public void setLeftMotorSpeed(double speed) {
    leftMotor.set(speed);
  }

  public void setRightMotorSpeed(double speed) {
    rightMotor.set(speed);
  }

  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
}
