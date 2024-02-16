// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.CANIDS;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax leftMotor; 
  private final CANSparkMax rightMotor; 
  private final CANcoder encoder;

  public ArmSubsystem() {
    leftMotor = new CANSparkMax(CANIDS.ARM_LEFT, MotorType.kBrushless);
    rightMotor = new CANSparkMax(CANIDS.ARM_RIGHT, MotorType.kBrushless);

    encoder = new CANcoder(CANIDS.ARM_ENCODER);
  }

  @Override
  public void periodic() {
    // Shuffleboard.getTab("CAN Bus")
    // .add("Arm Left", leftMotor.getBusVoltage());

    //  Shuffleboard.getTab("CAN Bus")
    // .add("Arm Right", rightMotor.getBusVoltage());
  }

  public double getArmPosition() {
    return encoder.getPosition().getValueAsDouble();
  }

  public double getAbsoluteArmPosition() {
    return encoder.getAbsolutePosition().getValueAsDouble();
  }

  public void setMotorSpeed(double speed) {
    leftMotor.set(speed);
    rightMotor.set(-speed);
  }

  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
}
