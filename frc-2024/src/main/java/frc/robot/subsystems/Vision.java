// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  private PhotonCamera m_cam = new PhotonCamera("photonvision"); //TODO: Change to actual camera name
  private Transform3d m_robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //TODO: Change this later!
  
  private PhotonPipelineResult m_result;
  private boolean m_hasTargets;
  private List<PhotonTrackedTarget> m_targets;
  private PhotonTrackedTarget m_bestTarget;

  private double m_yaw;
  private double m_pitch;
  private double m_area;
  private double m_skew;

  private Transform3d m_camToTarget;
  private List<TargetCorner> m_corners;
  private int m_ID;
  private double m_poseAmbiguity;

  private AprilTagFieldLayout m_aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private Pose3d m_robotPose;
  /** Creates a new Vision. */
  public Vision() {
  }

  private double getYaw(){
    return m_yaw;
  }

  private double getPitch(){
    return m_pitch;
  }

  private double getArea(){
    return m_area;
  }

  private double getSkew(){
    return m_skew;
  }

  private Transform3d getCameraToTarget(){
    return m_camToTarget;
  }

  private List<TargetCorner> getCorners(){
    return m_corners;
  }

  private int getID(){
    return m_ID;
  }

  private double getPoseAmbiguity(){
    return m_poseAmbiguity;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_result = m_cam.getLatestResult();
    m_hasTargets = m_result.hasTargets();
    m_targets = m_result.getTargets();
    m_bestTarget = m_result.getBestTarget();

    m_yaw = m_bestTarget.getYaw();
    m_pitch = m_bestTarget.getPitch();
    m_area = m_bestTarget.getArea();
    m_skew = m_bestTarget.getSkew();

    m_camToTarget = m_bestTarget.getBestCameraToTarget();
    m_corners = m_bestTarget.getDetectedCorners();
    m_ID = m_bestTarget.getFiducialId();
    m_poseAmbiguity = m_bestTarget.getPoseAmbiguity();

    m_robotPose = PhotonUtils.estimateFieldToRobotAprilTag(m_camToTarget, m_aprilTagFieldLayout.getTagPose(m_ID).get(), m_robotToCam);
  }
}
