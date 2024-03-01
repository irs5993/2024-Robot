// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.helpers.RMath;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ShootDistanceCommand extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final VisionSubsystem visionSubsystem;

  private double topSpeed = 0.7;
  private double bottomSpeed = 0.7;

  public ShootDistanceCommand(ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem) {
    addRequirements(shooterSubsystem);

    this.shooterSubsystem = shooterSubsystem;
    this.visionSubsystem = visionSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var target = visionSubsystem.getSpeakerTarget();
    if (target != null) {
      double distance = visionSubsystem.getTargetDistance(target, visionSubsystem.SPEAKER_APRILTAG_HEIGHT_METERS);

      topSpeed = RMath.map(distance, 0.5, 3, 0.57, 0.85);
      bottomSpeed = RMath.map(distance, 0.5, 3, 0.57, 0.9);
    }

    this.shooterSubsystem.setTopMotorSpeed(topSpeed);
    this.shooterSubsystem.setBottomMotorSpeed(bottomSpeed);

    SmartDashboard.putNumber("TOP SPEED", topSpeed);
    SmartDashboard.putNumber("BOTTOM SPEED", bottomSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.shooterSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
