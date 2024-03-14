// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class CalculateArmVisionCommand extends Command {
  private final ArmSubsystem armSubsystem;
  private final VisionSubsystem visionSubsystem;

  private double latestAngle;

  public CalculateArmVisionCommand(ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem) {
    addRequirements(armSubsystem);

    this.armSubsystem = armSubsystem;
    this.visionSubsystem = visionSubsystem;
  }

  // Komut başlangıçta planlandığında çağrılır.
  @Override
  public void initialize() {
    armSubsystem.resetController();

    latestAngle = armSubsystem.getAngle();
  }

  // Komut planlanırken zamanlayıcı her çalıştığında çağrılır.
  @Override
  public void execute() {
    var target = visionSubsystem.getSpeakerTarget();

    double desiredAngle = computeAngle(target);
    latestAngle = desiredAngle;

    SmartDashboard.putNumber("Desired Angle", desiredAngle);

    double desiredPosition = armSubsystem.angleToPosition(desiredAngle);
    armSubsystem.setPosition(desiredPosition);
  }

  public double computeAngle(PhotonTrackedTarget target) {
    if (target == null) {
      return latestAngle;
    }

    double currentAngle = armSubsystem.getAngle();

    double h = 1.825 - 0.6 * Math.sin(Math.toRadians(currentAngle));
    double d = 0.135 + 0.6 * Math.cos(Math.toRadians(currentAngle)) + (visionSubsystem
        .getTargetDistance(target, visionSubsystem.SPEAKER_APRILTAG_HEIGHT_METERS));

    double angle = 90 - (Math.toDegrees(Math.atan(h / d)) + 28.5);

    return angle + Constants.Arm.ANGLE_OFFSET;
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
