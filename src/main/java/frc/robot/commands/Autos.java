// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.commands.arm.AdjustArmVisionCommand;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {

  public static Command hayalGucunuKullan(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem,
      ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem, ConveyorSubsystem conveyorSubsystem) {
    return Commands.sequence(

        new CenterTargetCommand(drivetrainSubsystem, visionSubsystem)
            .alongWith(new AdjustArmVisionCommand(armSubsystem, visionSubsystem)),

        new ShootCommand(shooterSubsystem, () -> 0.6, () -> 0.6).withTimeout(0.5),

        new DynamicDriveCommand(drivetrainSubsystem, () -> -0.35, () -> -0, () -> 0).withTimeout(1.5),

        new SetArmPositionCommand(armSubsystem,
            () -> Constants.Arm.MIN_POSITION),

        new RunConveyorCommand(conveyorSubsystem, 0.6).withTimeout(0.73),

        new DynamicDriveCommand(drivetrainSubsystem, () -> 0.35, () -> -0, () -> 0).withTimeout(1.5),

        new CenterTargetCommand(drivetrainSubsystem, visionSubsystem)
            .alongWith(new AdjustArmVisionCommand(armSubsystem, visionSubsystem)),

        new ShootCommand(shooterSubsystem, () -> 0.6, () -> 0.6)).withTimeout(0.84);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
