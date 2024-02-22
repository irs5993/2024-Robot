// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class CenterTargetCommand extends Command {
  private final VisionSubsystem visionSubsystem;
  private final DrivetrainSubsystem drivetrainSubsystem;

  private final double MAX_OUT = 0.6;
  private final PIDController pid = new PIDController(0.05, 0, 0.01);

  public CenterTargetCommand(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem,
      int pipelineIndex) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.visionSubsystem = visionSubsystem;
    addRequirements(visionSubsystem, drivetrainSubsystem);

    visionSubsystem.setPipelineIndex(pipelineIndex);

    pid.setTolerance(3);
    pid.enableContinuousInput(-180, 180);
  }

  @Override
  public void execute() {
    var target = visionSubsystem.getBestTarget();
    double rotation = 0;

    if (target != null) {
      rotation = MathUtil.clamp(-pid.calculate(target.getYaw()), -MAX_OUT, MAX_OUT);
    }

    drivetrainSubsystem.drive(0, rotation);
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