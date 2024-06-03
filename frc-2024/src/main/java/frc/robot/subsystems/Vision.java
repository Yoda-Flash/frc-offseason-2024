// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  private PhotonCamera m_cam = new PhotonCamera("photonvision"); //TODO: Change to actual camera name
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

  /** Creates a new Vision. */
  public Vision() {
  }

  private double getYaw(){
    m_yaw = m_bestTarget.getYaw();
    return m_yaw;
  }

  private double getPitch(){
    m_pitch = m_bestTarget.getPitch();
    return m_pitch;
  }

  private double getArea(){
    m_area = m_bestTarget.getArea();
    return m_area;
  }

  private double getSkew(){
    m_skew = m_bestTarget.getSkew();
    return m_skew;
  }

  private Transform3d getCameraToTarget(){
    m_camToTarget = m_bestTarget.getBestCameraToTarget();
    return m_camToTarget;
  }

  private List<TargetCorner> getCorners(){
    m_corners = m_bestTarget.getDetectedCorners();
    return m_corners;
  }

  private int getID(){
    m_ID = m_bestTarget.getFiducialId();
    return m_ID;
  }

  private double getPoseAmbiguity(){
    m_poseAmbiguity = m_bestTarget.getPoseAmbiguity();
    return m_poseAmbiguity;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_result = m_cam.getLatestResult();
    m_hasTargets = m_result.hasTargets();
    m_targets = m_result.getTargets();
    m_bestTarget = m_result.getBestTarget();
  }
}
