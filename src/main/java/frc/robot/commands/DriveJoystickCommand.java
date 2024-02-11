// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveJoystickCommand extends Command {
  /** Creates a new DriveJoystickCommand. */
  private final CommandJoystick joystick = new CommandJoystick(0);
  private final DrivetrainSubsystem drivetrainSubsystem;
  private double xSpeed, zRotation;
  
  public DriveJoystickCommand(DrivetrainSubsystem drivetrainSubsystem, double xSpeed, double zRotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);

    xSpeed = joystick.getX();
    zRotation = joystick.getZ();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
