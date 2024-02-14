// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;

public class RunConveyorCommand extends Command {
  private final ConveyorSubsystem conveyorSubsystem;
  private double speed;

  public RunConveyorCommand(ConveyorSubsystem conveyorSubsystem, double speed) {
    addRequirements(conveyorSubsystem);

    this.conveyorSubsystem = conveyorSubsystem;
    this.speed = speed;
  }

  @Override
  public void execute() {
    this.conveyorSubsystem.setMotorSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    this.conveyorSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
