// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class SetArmPositionCommand extends Command {
  private final ArmSubsystem armSubsystem;
  private final DoubleSupplier positionSupplier;

  public SetArmPositionCommand(ArmSubsystem armSubsystem, DoubleSupplier positionSupplier) {
    addRequirements(armSubsystem);

    this.armSubsystem = armSubsystem;
    this.positionSupplier = positionSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.resetController();
  }

  @Override
  public void execute() {
    armSubsystem.setPosition(positionSupplier.getAsDouble());
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
