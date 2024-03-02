// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class CenterNoteCommand extends Command {
  private final VisionSubsystem visionSubsystem;
  private final DrivetrainSubsystem drivetrainSubsystem;

  private final double MAX_OUT = 0.8;
  private final PIDController pid = new PIDController(0.08, 0.02, 0);

  public CenterNoteCommand(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem) {
    addRequirements(drivetrainSubsystem);

    this.drivetrainSubsystem = drivetrainSubsystem;
    this.visionSubsystem = visionSubsystem;

    pid.setTolerance(0.1);
    pid.enableContinuousInput(-180, 180);
  }

  @Override
  public void execute() {
    var target = visionSubsystem.getBestTargetIntake();
    double rotation = 0;

    if (target != null) {
      rotation = MathUtil.clamp(-pid.calculate(target.getYaw(), 0), -MAX_OUT, MAX_OUT);
      SmartDashboard.putNumber("CURRENT YAW", target.getYaw());

    }

    drivetrainSubsystem.drive(0, rotation);

    SmartDashboard.putNumber("TARGET ROTATION", 0);

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