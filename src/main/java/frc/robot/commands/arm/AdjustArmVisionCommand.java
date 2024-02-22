// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AdjustArmVisionCommand extends Command {
  private final ArmSubsystem armSubsystem;
  private final VisionSubsystem visionSubsystem;
  private Command armPositionCommand;
  private double latestAngle = 45;

  public AdjustArmVisionCommand(ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem) {
    addRequirements(visionSubsystem);

    this.armPositionCommand = new SetArmPositionOTGCommand(armSubsystem,
        () -> armSubsystem.angleToEncoderPosition(computeAngle().getAsDouble()));

    this.armSubsystem = armSubsystem;
    this.visionSubsystem = visionSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("STARTED RUNNING ADJUST");
    CommandScheduler.getInstance().schedule(armPositionCommand);
  }

  // Called every time the scheduler runs while the command is scheduled.
  public DoubleSupplier computeAngle() {

    var bestTarget = visionSubsystem.getBestTarget();
    if (bestTarget == null) {
      return () -> latestAngle;
    }

    double armAngle = armSubsystem.getAngle();

    double h = 1.77 - 0.6 * Math.sin(Math.toRadians(armAngle));
    double d = 0.12 + 0.6 * Math.cos(Math.toRadians(armAngle)) + (visionSubsystem
        .getTargetDistance(bestTarget, visionSubsystem.SPEAKER_APRILTAG_HEIGHT_METERS));

    DoubleSupplier angle = () -> 90 - (Math.toDegrees(Math.atan(h / d)) + 28.5);

    SmartDashboard.putNumber("h", h);
    SmartDashboard.putNumber("d", d);
    SmartDashboard.putNumber("angle", angle.getAsDouble());
    SmartDashboard.putNumber("encoder pos", armSubsystem.angleToEncoderPosition(angle.getAsDouble()));

    latestAngle = angle.getAsDouble();
    return angle;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CommandScheduler.getInstance().cancel(armPositionCommand);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
