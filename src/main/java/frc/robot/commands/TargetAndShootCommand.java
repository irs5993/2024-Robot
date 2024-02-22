package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.arm.AdjustArmVisionCommand;
import frc.robot.commands.arm.SetArmPositionLongRangeCommand;
import frc.robot.commands.drive.TurnAngleCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class TargetAndShootCommand extends SequentialCommandGroup {
  public TargetAndShootCommand(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem,
      ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem) {
    addCommands(
        new CenterTargetCommand(drivetrainSubsystem, visionSubsystem, Constants.Vision.PIPELINE_APRILTAG),
        new AdjustArmVisionCommand(armSubsystem, visionSubsystem));
  }
}

// @Override
// public void initialize() {
// PhotonTrackedTarget target = visionSubsystem.getBestTarget();
// int targetID = target.getFiducialId();

// if (target != null && (targetID == 4 || targetID == 7)) {
// double distance = PhotonUtils.calculateDistanceToTargetMeters(
// visionSubsystem.CAMERA_HEIGHT_METERS,
// visionSubsystem.SPEAKER_APRILTAG_HEIGHT_METERS,
// visionSubsystem.CAMERA_PITCH_RADIANS,
// Units.degreesToRadians(target.getPitch()));
// }
// }
// }