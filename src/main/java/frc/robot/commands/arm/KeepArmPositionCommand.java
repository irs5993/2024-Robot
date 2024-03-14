// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class KeepArmPositionCommand extends Command {
  private final ArmSubsystem armSubsystem;
  private double targetPosition;

  public KeepArmPositionCommand(ArmSubsystem armSubsystem) {
    addRequirements(armSubsystem);

    this.armSubsystem = armSubsystem;
  }

  // Komut başlangıçta planlandığında çağrılır.
  @Override
  public void initialize() {
    armSubsystem.resetController();
    armSubsystem.setControllerPID(8, 0.1, 0.7);

    targetPosition = armSubsystem.getAbsolutePosition();
  }

  // Komut planlanırken zamanlayıcı her çalıştığında çağrılır.
  @Override
  public void execute() {
    armSubsystem.setPosition(targetPosition);
  }

  // Komut sona erdiğinde veya kesintiye uğradığında çağrılır.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.stop();
  }

  // Komutun bitmesi gerektiğinde true değerini döndürür.
  @Override
  public boolean isFinished() {
    return false;
  }
}
