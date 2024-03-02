// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootVelocityCommand extends Command {
  private final ShooterSubsystem shooterSubsystem;

  private final DoubleSupplier topVelocitySupplier;
  private final DoubleSupplier bottomVelocitySupplier;

  public ShootVelocityCommand(ShooterSubsystem shooterSubsystem, DoubleSupplier bottomVelocitySupplier,
      DoubleSupplier topVelocitySupplier) {
    addRequirements(shooterSubsystem);

    this.shooterSubsystem = shooterSubsystem;

    this.topVelocitySupplier = topVelocitySupplier;
    this.bottomVelocitySupplier = bottomVelocitySupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.shooterSubsystem.setBottomMotorVelocity(bottomVelocitySupplier.getAsDouble());
    this.shooterSubsystem.setTopMotorVelocity(topVelocitySupplier.getAsDouble());
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
