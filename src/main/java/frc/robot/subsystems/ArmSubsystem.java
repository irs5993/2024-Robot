// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.CANIDS;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax leftMotor; // <- Lead
  private final CANSparkMax rightMotor; // <- Follower
  private final CANcoder encoder;

  public ArmSubsystem() {
    leftMotor = new CANSparkMax(CANIDS.ARM_LEFT, MotorType.kBrushless);
    rightMotor = new CANSparkMax(CANIDS.ARM_RIGHT, MotorType.kBrushless);
    rightMotor.follow(leftMotor);

    encoder = new CANcoder(CANIDS.ARM_ENCODER);
  }

  @Override
  public void periodic() {
    System.out.println(encoder.getPosition());
  }

  public double getArmPosition() {
    return encoder.getPosition().getValueAsDouble();
  }

  public double getAbsoluteArmPosition() {
    return encoder.getAbsolutePosition().getValueAsDouble();
  }

  public void setMotorSpeed(double speed) {
    leftMotor.set(speed);
  }

  public void stop() {
    leftMotor.stopMotor();
  }
}
