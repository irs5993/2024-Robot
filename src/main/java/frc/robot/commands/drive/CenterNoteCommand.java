// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class CenterNoteCommand extends Command {
  private final VisionSubsystem visionSubsystem;
  private final DrivetrainSubsystem drivetrainSubsystem;

  private final double MAX_OUT = 0.5;
  private final PIDController pid = new PIDController(0.08, 0.02, 0);

  public CenterNoteCommand(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem) {
    addRequirements(drivetrainSubsystem);

    this.drivetrainSubsystem = drivetrainSubsystem;
    this.visionSubsystem = visionSubsystem;

    pid.setTolerance(0.1);
  }

  @Override
  public void execute() {
    var target = visionSubsystem.getBestTargetIntake();

    if (target != null) {
      double rotation = MathUtil.clamp(-pid.calculate(target.getYaw(), 1.5), -MAX_OUT, MAX_OUT);
      drivetrainSubsystem.drive(0, rotation);
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