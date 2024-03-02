// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class CenterTargetCommand extends Command {
  private final VisionSubsystem visionSubsystem;
  private final DrivetrainSubsystem drivetrainSubsystem;

  private final double MAX_OUT = 0.8;
  private final PIDController pid = new PIDController(0.1, 0.02, 0);

  public CenterTargetCommand(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem) {
    addRequirements(drivetrainSubsystem);

    this.drivetrainSubsystem = drivetrainSubsystem;
    this.visionSubsystem = visionSubsystem;

    pid.setTolerance(0.1);

    SmartDashboard.putNumber("Center Target Desired Position", 0);
  }

  @Override
  public void execute() {
    var target = visionSubsystem.getSpeakerTarget();

    if (target != null) {
      double rotation = MathUtil.clamp(-pid.calculate(target.getYaw(), 0), -MAX_OUT, MAX_OUT);
      drivetrainSubsystem.drive(0, rotation);

      SmartDashboard.putNumber("Center Target Current Yaw", target.getYaw());

    } else {
      drivetrainSubsystem.stop();
    }

  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}