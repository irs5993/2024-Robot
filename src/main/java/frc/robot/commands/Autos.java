// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.commands.arm.CalculateArmVisionCommand;
import frc.robot.commands.arm.MoveArmVisionCommand;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.commands.drive.CenterTargetCommand;
import frc.robot.commands.drive.DriveCenterNoteCommand;
import frc.robot.commands.drive.DynamicDriveCommand;
import frc.robot.commands.shoot.ShootCommand;
import frc.robot.commands.shoot.ShootDistanceCommand;
import frc.robot.commands.shoot.ShootVelocityCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  public static Command CenterAuto(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem,
      ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem, ConveyorSubsystem conveyorSubsystem) {
    return Commands.sequence(

        new SetArmPositionCommand(armSubsystem, () -> Constants.Arm.DEFAULT_SHOOT_POSITION).withTimeout(2)
            .alongWith(new ShootVelocityCommand(shooterSubsystem, () -> 0.63, () -> 0.63).withTimeout(2.5)),

        new RunConveyorCommand(conveyorSubsystem, -0.8).withTimeout(1),

        new SetArmPositionCommand(armSubsystem, () -> Constants.Arm.MIN_POSITION).withTimeout(1.3),

        new DynamicDriveCommand(drivetrainSubsystem, () -> 0.6, () -> 0, () -> 1).withTimeout(1.5)
            .alongWith(new RunConveyorCommand(conveyorSubsystem, -0.52).withTimeout(1.5)),

        new RunConveyorCommand(conveyorSubsystem, 0.3)
            .alongWith(new ShootCommand(shooterSubsystem, () -> -0.15, () -> -0.15))
            .alongWith(new DynamicDriveCommand(drivetrainSubsystem, () -> -0.5, () -> 0, () -> 1)).withTimeout(0.3),

        new MoveArmVisionCommand(armSubsystem, visionSubsystem)
            .alongWith(new ShootVelocityCommand(shooterSubsystem, () -> 0.6, () -> 0.6)).withTimeout(2.5),

        new RunConveyorCommand(conveyorSubsystem, -0.8).withTimeout(1)

    );
  }

  public static Command RightAuto(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem,
      ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem, ConveyorSubsystem conveyorSubsystem) {
    return Commands.sequence(

        new SetArmPositionCommand(armSubsystem, () -> 0.027).withTimeout(2)
            .alongWith(new ShootVelocityCommand(shooterSubsystem, () -> 0.63, () -> 0.63).withTimeout(2.5)),

        new RunConveyorCommand(conveyorSubsystem, -0.4).withTimeout(1),

        new SetArmPositionCommand(armSubsystem, () -> Constants.Arm.MIN_POSITION).withTimeout(1.3)
            .alongWith(new DynamicDriveCommand(drivetrainSubsystem, () -> 0.6, () -> 0.7, () -> 1).withTimeout(2.5)),

        new DriveCenterNoteCommand(drivetrainSubsystem, visionSubsystem, 0.6)
            .alongWith(new RunConveyorCommand(conveyorSubsystem, -0.52)).withTimeout(2.3),

        new RunConveyorCommand(conveyorSubsystem, 0.3)
            .alongWith(new ShootCommand(shooterSubsystem, () -> -0.15, () -> -0.15))
            .alongWith(new DynamicDriveCommand(drivetrainSubsystem, () -> -0.5, () -> 0, () -> 1)).withTimeout(0.3),

        new DynamicDriveCommand(drivetrainSubsystem, () -> -0.6, () -> 0, () -> 1).withTimeout(0.7),

        new DynamicDriveCommand(drivetrainSubsystem, () -> 0, () -> -0.65, () -> 1)
            .until(() -> visionSubsystem.getSpeakerTarget() != null),

        new MoveArmVisionCommand(armSubsystem, visionSubsystem)
            .alongWith(new ShootDistanceCommand(shooterSubsystem, visionSubsystem))
            .alongWith(new CenterTargetCommand(drivetrainSubsystem, visionSubsystem)).withTimeout(3)
            .alongWith(new WaitCommand(2).andThen(new RunConveyorCommand(conveyorSubsystem, -0.4).withTimeout(2))));

  }

  public static Command LeftAuto(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem,
      ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem, ConveyorSubsystem conveyorSubsystem) {
    return Commands.sequence(

        new SetArmPositionCommand(armSubsystem, () -> 0.027).withTimeout(2)
            .alongWith(new ShootVelocityCommand(shooterSubsystem, () -> 0.63, () -> 0.63).withTimeout(2.5)),

        new RunConveyorCommand(conveyorSubsystem, -0.4).withTimeout(1),

        new SetArmPositionCommand(armSubsystem, () -> Constants.Arm.MIN_POSITION).withTimeout(1.3)
            .alongWith(new DynamicDriveCommand(drivetrainSubsystem, () -> 0.6, () -> -0.7, () -> 1).withTimeout(2.5)),

        new DriveCenterNoteCommand(drivetrainSubsystem, visionSubsystem, 0.6)
            .alongWith(new RunConveyorCommand(conveyorSubsystem, -0.52)).withTimeout(2.3),

        new RunConveyorCommand(conveyorSubsystem, 0.3)
            .alongWith(new ShootCommand(shooterSubsystem, () -> -0.15, () -> -0.15))
            .alongWith(new DynamicDriveCommand(drivetrainSubsystem, () -> -0.5, () -> 0, () -> 1)).withTimeout(0.3),

        new DynamicDriveCommand(drivetrainSubsystem, () -> -0.6, () -> 0, () -> 1).withTimeout(0.7),

        new DynamicDriveCommand(drivetrainSubsystem, () -> 0, () -> 0.65, () -> 1)
            .until(() -> visionSubsystem.getSpeakerTarget() != null),

        new MoveArmVisionCommand(armSubsystem, visionSubsystem)
            .alongWith(new ShootDistanceCommand(shooterSubsystem, visionSubsystem))
            .alongWith(new CenterTargetCommand(drivetrainSubsystem, visionSubsystem)).withTimeout(3)
            .alongWith(new WaitCommand(2).andThen(new RunConveyorCommand(conveyorSubsystem, -0.4).withTimeout(2))));

  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
