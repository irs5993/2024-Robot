// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmCommand extends Command {
  private final ArmSubsystem armSubsystem;
  private double speed;

  // Constructor
  public MoveArmCommand(ArmSubsystem armSubsystem, double speed) {
    addRequirements(armSubsystem);

    this.armSubsystem = armSubsystem;
    this.speed = speed;
  }

  // Komut planlanırken zamanlayıcı her çalıştığında çağrılır.
  @Override
  public void execute() {
    this.armSubsystem.setMotorSpeed(speed);
  }

  // Komut sona erdiğinde veya kesintiye uğradığında çağrılır.
  @Override
  public void end(boolean interrupted) {
    this.armSubsystem.stop();
  }

  // Komutun bitmesi gerektiğinde true değerini döndürür.
  @Override
  public boolean isFinished() {
    return false; // Keep the command running until explicitly interrupted or canceled
  }
}
