// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Collections;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  public PhotonCamera main_camera;
  public PhotonCamera intake_camera;

  public final double CAMERA_HEIGHT_METERS = 0.235; // Robotta kamera yüksekliği
  public final double SPEAKER_APRILTAG_HEIGHT_METERS = 1.45; // AprilTag yüksekliği
  public final double SUB_RANGE_METERS = 0.95; // Sahadaki subwoofer aralığı
  public final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(37); // Robotta kamera açısı

  public VisionSubsystem() {
    main_camera = new PhotonCamera("main");
    intake_camera = new PhotonCamera("intake");
  }

  @Override
  public void periodic() {

  }

  public PhotonTrackedTarget getBestTargetIntake() {
    if (!intake_camera.isConnected()) {
      return null;
    }

    var result = getLatestResultIntake();
    if (result.hasTargets()) {
      return result.getBestTarget();
    }
    return null;
  }

  public void setPipelineIndex(int index) {
    if (!main_camera.isConnected()) {
      return;
    }
    main_camera.setPipelineIndex(index);
  }

  public PhotonPipelineResult getLatestResult() {
    if (!main_camera.isConnected()) {
      return null;
    }
    return main_camera.getLatestResult();
  }

  public PhotonPipelineResult getLatestResultIntake() {
    if (!intake_camera.isConnected()) {
      return null;
    }
    return intake_camera.getLatestResult();
  }

  public List<PhotonTrackedTarget> getTargets() {
    if (!main_camera.isConnected()) {
      return Collections.emptyList();
    }
    return getLatestResult().getTargets();
  }

  public PhotonTrackedTarget getBestTarget() {
    if (!main_camera.isConnected()) {
      return null;
    }

    var result = getLatestResult();
    if (result.hasTargets()) {
      return result.getBestTarget();
    }
    return null;
  }

  public PhotonTrackedTarget getSpeakerTarget() {
    var targets = getTargets();
    PhotonTrackedTarget target = null;

    for (int i = 0; i < targets.size(); i++) {
      if (targets.get(i).getFiducialId() == 4 || targets.get(i).getFiducialId() == 7) {
        target = targets.get(i);
        break;
      }
    }

    return target;
  }

  public double getTargetDistance(PhotonTrackedTarget target, double targetHeightMeters) {
    return PhotonUtils.calculateDistanceToTargetMeters(
        CAMERA_HEIGHT_METERS,
        targetHeightMeters,
        CAMERA_PITCH_RADIANS,
        Units.degreesToRadians(target.getPitch()));
  }
}
