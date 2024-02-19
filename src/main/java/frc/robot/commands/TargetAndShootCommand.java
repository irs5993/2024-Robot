package frc.robot.commands;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class TargetAndShootCommand extends Command {

  private final VisionSubsystem visionSubsystem;
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final ArmSubsystem armSubsystem;

  public TargetAndShootCommand(VisionSubsystem visionSubsystem,
      DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem) {
    addRequirements(drivetrainSubsystem, armSubsystem, visionSubsystem);

    this.visionSubsystem = visionSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.armSubsystem = armSubsystem;

  }

  @Override
  public void initialize() {
    PhotonTrackedTarget target = visionSubsystem.getBestTarget();
    int targetID = target.getFiducialId();

    if (target != null && (targetID == 4 || targetID == 7)) {
      double distance = PhotonUtils.calculateDistanceToTargetMeters(
          visionSubsystem.CAMERA_HEIGHT_METERS,
          visionSubsystem.SPEAKER_APRILTAG_HEIGHT_METERS,
          visionSubsystem.CAMERA_PITCH_RADIANS,
          Units.degreesToRadians(target.getPitch()));
    }
  }
}