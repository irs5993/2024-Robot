package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.commands.drive.TurnAngleCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;



  public class TargetAndShootCommand extends SequentialCommandGroup {
    public TargetAndShootCommand(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem) {
      addCommands(
        new ParallelCommandGroup(
          new TurnAngleCommand(drivetrainSubsystem, () -> 0),
          new SetArmPositionCommand(armSubsystem, () -> 0)
        )
      );
    }
  }

  // @Override
  // public void initialize() {
  //   PhotonTrackedTarget target = visionSubsystem.getBestTarget();
  //   int targetID = target.getFiducialId();

  //   if (target != null && (targetID == 4 || targetID == 7)) {
  //     double distance = PhotonUtils.calculateDistanceToTargetMeters(
  //         visionSubsystem.CAMERA_HEIGHT_METERS,
  //         visionSubsystem.SPEAKER_APRILTAG_HEIGHT_METERS,
  //         visionSubsystem.CAMERA_PITCH_RADIANS,
  //         Units.degreesToRadians(target.getPitch()));
  //   }
  // }
// }