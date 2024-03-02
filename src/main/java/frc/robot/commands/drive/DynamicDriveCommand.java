// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.helpers.RMath;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DynamicDriveCommand extends Command {

  private final DrivetrainSubsystem drivetrainSubsystem;
  private final DoubleSupplier xSpeedSupplier, zRotationSupplier, multiplierSupplier;

  /** Creates a new DynamicDriveCommand. */
  public DynamicDriveCommand(DrivetrainSubsystem drivetrainSubsystem, DoubleSupplier xSpeedSupplier,
      DoubleSupplier zRotationSupplier, DoubleSupplier multiplierSupplier) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);

    this.xSpeedSupplier = xSpeedSupplier;
    this.zRotationSupplier = zRotationSupplier;
    this.multiplierSupplier = multiplierSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = xSpeedSupplier.getAsDouble() * multiplierSupplier.getAsDouble();
    double zRotation = zRotationSupplier.getAsDouble() * multiplierSupplier.getAsDouble();

    drivetrainSubsystem.drive(xSpeed, zRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
