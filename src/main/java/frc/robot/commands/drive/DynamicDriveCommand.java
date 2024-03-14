// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DynamicDriveCommand extends Command {

  private final DrivetrainSubsystem drivetrainSubsystem;
  private final DoubleSupplier xSpeedSupplier, zRotationSupplier, multiplierSupplier;

  /** Yeni bir DynamicDriveCommand yaratır. */
  public DynamicDriveCommand(DrivetrainSubsystem drivetrainSubsystem, DoubleSupplier xSpeedSupplier,
      DoubleSupplier zRotationSupplier, DoubleSupplier multiplierSupplier) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);

    this.xSpeedSupplier = xSpeedSupplier;
    this.zRotationSupplier = zRotationSupplier;
    this.multiplierSupplier = multiplierSupplier;
  }

  // Komut başlangıçta programlandığında çağrılır.
  @Override
  public void initialize() {
  }

  // Komut planlanırken zamanlayıcı her çalıştığında çağrılır.
  @Override
  public void execute() {
    double xSpeed = xSpeedSupplier.getAsDouble() * multiplierSupplier.getAsDouble();
    double zRotation = zRotationSupplier.getAsDouble() * multiplierSupplier.getAsDouble();

    drivetrainSubsystem.drive(xSpeed, zRotation);
  }

  // Komut bittiğinde veya kesintiye uğradığında çağrılır.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

 // Komutun bitmesi gerektiğinde true değerini döndürür.
  @Override
  public boolean isFinished() {
    return false;
  }
}
