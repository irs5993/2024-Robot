// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
   public static Command hayalGucunuKullan(DrivetrainSubsystem drivetrainSubsystem) {
    return Commands.sequence(
      
    new DynamicDriveCommand(drivetrainSubsystem, () -> 0.2, () -> -0.1, () -> 0).withTimeout(1.5),
    new DynamicDriveCommand(drivetrainSubsystem, () -> -0.3, () -> 0.2, () -> 0).withTimeout(3)
    );
  } 

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
