// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AdjustArmVisionCommand extends Command {
  private final ArmSubsystem armSubsystem;
  private final VisionSubsystem visionSubsystem;

  private double latestAngle = 45;

  public AdjustArmVisionCommand(ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem) {
    addRequirements(armSubsystem);

    this.armSubsystem = armSubsystem;
    this.visionSubsystem = visionSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double desiredAngle = computeAngle();
    latestAngle = desiredAngle;

    SmartDashboard.putNumber("Desired Angle", desiredAngle);

    double desiredPosition = armSubsystem.angleToPosition(desiredAngle);

    armSubsystem.resetController();
    armSubsystem.setPosition(desiredPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  public double computeAngle() {
    var bestTarget = visionSubsystem.getBestTarget();
    if (bestTarget == null) {
      return latestAngle;
    }

    double currentAngle = armSubsystem.getAngle();

    double h = 1.77 - 0.6 * Math.sin(Math.toRadians(currentAngle));
    double d = 0.12 + 0.6 * Math.cos(Math.toRadians(currentAngle)) + (visionSubsystem
        .getTargetDistance(bestTarget, visionSubsystem.SPEAKER_APRILTAG_HEIGHT_METERS));

    double angle = 90 - (Math.toDegrees(Math.atan(h / d)) + 28.5);

    return angle;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
