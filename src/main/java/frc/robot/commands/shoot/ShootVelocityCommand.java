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

  // Komut başlangıçta programlandığında çağrılır.
  @Override
  public void initialize() {
  }

  // Komut planlanırken zamanlayıcı her çalıştığında çağrılır.
  @Override
  public void execute() {
    this.shooterSubsystem.setBottomMotorVelocity(bottomVelocitySupplier.getAsDouble());
    this.shooterSubsystem.setTopMotorVelocity(topVelocitySupplier.getAsDouble());
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
