// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
  public PhotonCamera camera;
  private final Servo servo;

  public final double CAMERA_HEIGHT_METERS = 0.22; // Camera height on robot
  public final double SPEAKER_APRILTAG_HEIGHT_METERS = 1.55; // AprilTag height
  public final double SUB_RANGE_METERS = 0.95; // Subwoofer range through the field
  public final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(45); // Camera angle on robot

  // Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
  // new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of
  // center, half a meter up from center.
  // AprilTagFieldLayout aprilTagFieldLayout =
  // AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  // PhotonPoseEstimator photonPoseEstimator = new
  // PhotonPoseEstimator(aprilTagFieldLayout,
  // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);

  public VisionSubsystem() {
    camera = new PhotonCamera("main");
    servo = new Servo(Constants.DriverPorts.CAMERA_SERVO);
  }

  @Override
  public void periodic() {
    List<PhotonTrackedTarget> targets = getTargets();

    if (!targets.isEmpty()) {
      // Create a list to store target distances
      double[] targetDistances = new double[targets.size()];

      // Calculate and store distances for each target
      for (int i = 0; i < targets.size(); i++) {
        PhotonTrackedTarget target = targets.get(i);
        double targetDistance = getTargetDistance(target, SPEAKER_APRILTAG_HEIGHT_METERS);
        targetDistances[i] = targetDistance;
      }

      // Display the list of target distances on SmartDashboard
      SmartDashboard.putNumberArray("Target Distances", targetDistances);
    }
  }

  public void setServoAngle(double angle) {
    servo.setAngle(angle);
  }

  public void setPipelineIndex(int index) {
    camera.setPipelineIndex(index);
  }

  public PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult();
  }

  public List<PhotonTrackedTarget> getTargets() {
    return getLatestResult().getTargets();
  }

  public PhotonTrackedTarget getBestTarget() {
    var result = getLatestResult();
    if (result.hasTargets()) {
      return result.getBestTarget();
    }
    return null;
  }

  public double getTargetDistance(PhotonTrackedTarget target, double targetHeightMeters) {
    return PhotonUtils.calculateDistanceToTargetMeters(
        CAMERA_HEIGHT_METERS,
        targetHeightMeters,
        CAMERA_PITCH_RADIANS,
        Units.degreesToRadians(target.getPitch()));
  }

  // public Optional<EstimatedRobotPose> gEstimatedGlobalPose(Pose2d
  // prevEstimatedRobotPose) {
  // photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
  // return photonPoseEstimator.update();
  // }
}
