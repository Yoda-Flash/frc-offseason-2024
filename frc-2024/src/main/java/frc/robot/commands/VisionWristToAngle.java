// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveDrive;

public class VisionSnapToAngle extends Command {

  private static final class Config{

  }

  private SwerveDrive m_swerve;
  // private double m_targetAngle;
  private double m_currentAngle;
  private double m_initAngle;
  private double m_turningSpeed;
  private double m_distance;
  private boolean m_detected;
  /** Creates a new VisionSnapToAngle. */
  public VisionSnapToAngle(SwerveDrive swerve) {
    m_swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_distance = SmartDashboard.getNumber("Distance", 0);
    System.out.println("Distance: " + m_distance);
    m_detected = SmartDashboard.getNumber("TagDetected",0);
    System.out.println("Detected: " + m_detected);
    // Construct chassis speed objects.
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, m_turningSpeed, m_wrist.getWristPosition());
    // Calculate module states.
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Set module states.
    m_swerve.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Stop");
    m_swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_currentAngle) <= Config.kDeadband;
  }
}
