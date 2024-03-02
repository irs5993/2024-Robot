// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.helpers.RMath;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ShootDistanceCommand extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final VisionSubsystem visionSubsystem;

  private double topVelocity = 0.65;
  private double bottomVelocity = 0.65;

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

      topVelocity = RMath.map(distance, 0.5, 3, 0.57, 0.8);
      bottomVelocity = RMath.map(distance, 0.5, 3, 0.57, 0.85);
    }

    this.shooterSubsystem.setTopMotorVelocity(topVelocity);
    this.shooterSubsystem.setBottomMotorVelocity(bottomVelocity);

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
