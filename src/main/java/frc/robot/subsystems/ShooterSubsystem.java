// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax topMotor;
  private final CANSparkMax bottomMotor;

  public ShooterSubsystem() {
    topMotor = new CANSparkMax(CANIDS.SHOOTER_TOP, MotorType.kBrushless);
    bottomMotor = new CANSparkMax(CANIDS.SHOOTER_BOTTOM, MotorType.kBrushless);
  }

  public void setShooterSpeed(double speed) {
    setTopMotorSpeed(speed);
    setBottomMotorSpeed(speed);
  }

  public void setTopMotorSpeed(double speed) {
    topMotor.set(speed);
  }

  public void setBottomMotorSpeed(double speed) {
    bottomMotor.set(speed);
  }

  public void stop() {
    topMotor.stopMotor();
    bottomMotor.stopMotor();
  }
}
