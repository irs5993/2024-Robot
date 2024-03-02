// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
  private final double MAX_RPM = 5700;

  private final CANSparkMax topMotor;
  private final CANSparkMax bottomMotor;

  private final SparkPIDController topPIDController;
  private final SparkPIDController bottomPIDController;

  private final RelativeEncoder topEncoder;
  private final RelativeEncoder bottomEncoder;

  public ShooterSubsystem() {
    topMotor = new CANSparkMax(CANIDS.SHOOTER_TOP, MotorType.kBrushless);
    bottomMotor = new CANSparkMax(CANIDS.SHOOTER_BOTTOM, MotorType.kBrushless);

    topMotor.restoreFactoryDefaults();
    bottomMotor.restoreFactoryDefaults();

    topPIDController = topMotor.getPIDController();
    bottomPIDController = bottomMotor.getPIDController();

    topEncoder = topMotor.getEncoder();
    bottomEncoder = bottomMotor.getEncoder();

    topPIDController.setP(6e-5);
    topPIDController.setI(0);
    topPIDController.setD(0);
    topPIDController.setIZone(0);
    topPIDController.setFF(0.00017);
    topPIDController.setOutputRange(-1, 1);

    bottomPIDController.setP(6e-5);
    bottomPIDController.setI(0);
    bottomPIDController.setD(0);
    bottomPIDController.setIZone(0);
    bottomPIDController.setFF(0.00017);
    bottomPIDController.setOutputRange(-1, 1);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("TOP VELOCITY", topEncoder.getVelocity());
    SmartDashboard.putNumber("BOTTOM VELOCITY", bottomEncoder.getVelocity());

  }

  public void setTopMotorVelocity(double velocity) {
    double setPoint = velocity * MAX_RPM;
    topPIDController.setReference(setPoint, ControlType.kVelocity);
  }

  public void setBottomMotorVelocity(double velocity) {
    double setPoint = velocity * MAX_RPM;
    bottomPIDController.setReference(setPoint, ControlType.kVelocity);
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
