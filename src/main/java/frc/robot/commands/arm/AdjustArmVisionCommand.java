// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AdjustArmVisionCommand extends Command {
  private final ArmSubsystem armSubsystem;
  private final VisionSubsystem visionSubsystem;

  private double latestAngle;

  public AdjustArmVisionCommand(ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem) {
    addRequirements(armSubsystem);

    this.armSubsystem = armSubsystem;
    this.visionSubsystem = visionSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.resetController();

    latestAngle = armSubsystem.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var target = visionSubsystem.getSpeakerTarget();

    double desiredAngle = computeAngle(target);
    latestAngle = desiredAngle;

    SmartDashboard.putNumber("Desired Angle", desiredAngle);

    double desiredPosition = armSubsystem.angleToPosition(desiredAngle);
    armSubsystem.setPosition(desiredPosition);
  }

  public double computeAngle(PhotonTrackedTarget target) {
    if (target == null) {
      return latestAngle;
    }

    double currentAngle = armSubsystem.getAngle();

    double h = 1.825 - 0.6 * Math.sin(Math.toRadians(currentAngle));
    double d = 0.135 + 0.6 * Math.cos(Math.toRadians(currentAngle)) + (visionSubsystem
        .getTargetDistance(target, visionSubsystem.SPEAKER_APRILTAG_HEIGHT_METERS));

    double angle = 90 - (Math.toDegrees(Math.atan(h / d)) + 28.5);

    return angle + Constants.Arm.ANGLE_OFFSET;
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
