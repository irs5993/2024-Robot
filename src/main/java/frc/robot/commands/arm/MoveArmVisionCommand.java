// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.helpers.RMath;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class MoveArmVisionCommand extends Command {
  private final ArmSubsystem armSubsystem;
  private final VisionSubsystem visionSubsystem;

  private double latestPosition;

  public MoveArmVisionCommand(ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem) {
    addRequirements(armSubsystem);

    this.armSubsystem = armSubsystem;
    this.visionSubsystem = visionSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.resetController();

    latestPosition = armSubsystem.getAbsolutePosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var target = visionSubsystem.getSpeakerTarget();
    double currentPosition;

    double position;
    if (target == null) {
      currentPosition = latestPosition;
    } else {
      currentPosition = target.getPitch();
      latestPosition = currentPosition;
    }

    position = RMath.map(currentPosition, -20, 20, 0.068, 0.015);

    // SmartDashboard.putNumber("Desired Angle", desiredAngle);

    // double desiredPosition = armSubsystem.angleToPosition(desiredAngle);
    armSubsystem.setPosition(position);
    SmartDashboard.putNumber("AMOGUS POSITION", position);
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