// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.CANIDS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax leftMotor; // <- Lead
  private final CANSparkMax rightMotor; // <- Follower

  public ArmSubsystem() {
    leftMotor = new CANSparkMax(CANIDS.ARM_LEFT, MotorType.kBrushless);
    rightMotor = new CANSparkMax(CANIDS.ARM_RIGHT, MotorType.kBrushless);

    rightMotor.follow(leftMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotorSpeed(double speed) {
    leftMotor.set(speed);
  }

  public void stop() {
    leftMotor.stopMotor();
  }
}
