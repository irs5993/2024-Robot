// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
  private final ShooterSubsystem shooterSubsystem;

  private final DoubleSupplier topSpeedSupplier;
  private final DoubleSupplier bottomSpeedSupplier;

  public ShootCommand(ShooterSubsystem shooterSubsystem, DoubleSupplier bottomSpeedSupplier,
      DoubleSupplier topSpeedSupplier) {
    addRequirements(shooterSubsystem);

    this.shooterSubsystem = shooterSubsystem;

    this.topSpeedSupplier = topSpeedSupplier;
    this.bottomSpeedSupplier = bottomSpeedSupplier;
  }

  // Komut başlangıçta programlandığında çağrılır.
  @Override
  public void initialize() {
  }

  // Komut planırken zamanlayıcı her çalıştığında çağrılır.
  @Override
  public void execute() {
    this.shooterSubsystem.setTopMotorSpeed(topSpeedSupplier.getAsDouble());
    this.shooterSubsystem.setBottomMotorSpeed(bottomSpeedSupplier.getAsDouble());
  }

  // Komut bittiğinde veya kesintiye uğradığında çağrılır.
  @Override
  public void end(boolean interrupted) {
    this.shooterSubsystem.stop();
  }

  // Komutun bitmesi gerektiğinde true değerini döndürür.
  @Override
  public boolean isFinished() {
    return false;
  }
}
