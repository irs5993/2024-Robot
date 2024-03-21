// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.helpers.RMath;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ShootDistanceCommand extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final VisionSubsystem visionSubsystem;

  private double latestPosition;

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
    double currentPosition;

    if (target == null) {
      currentPosition = latestPosition;
    } else {
      currentPosition = target.getPitch();
      latestPosition = currentPosition;
    }

    double topVelocity = RMath.map(currentPosition, -20, 20, 0.8, 0.6);
    double bottomVelocity = RMath.map(currentPosition, -20, 20, 0.83, 0.64);

    this.shooterSubsystem.setTopMotorVelocity(topVelocity);
    this.shooterSubsystem.setBottomMotorVelocity(bottomVelocity);

    SmartDashboard.putNumber("Shoot Distance Bottom Velocity", bottomVelocity);
    SmartDashboard.putNumber("Shoot Distance Top Velocity", topVelocity);

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
