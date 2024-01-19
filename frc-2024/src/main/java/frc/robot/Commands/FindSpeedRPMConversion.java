// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Flywheel;

public class FindSpeedRPMConversion extends Command {
  private static final class Config {
      private static final double kmotorSpeed = 0.1;
      private static final int kticksPerRevolution = 2048; //check if this is right
    }
    
  private Flywheel m_flywheel;
  private Timer m_timer = new Timer();
  private double m_initialPosition;
  /** Creates a new findSpeedRPMConversion. */
  public FindSpeedRPMConversion(Flywheel flywheel) {
    m_flywheel = flywheel;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_initialPosition = m_flywheel.getPosition();
    m_timer.start();
    SmartDashboard.putNumber("initial position", m_flywheel.getPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_flywheel.shoot(Config.kmotorSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_flywheel.stopShooting();
    m_timer.stop();
    m_timer.reset();
    SmartDashboard.putNumber("final position", m_flywheel.getPosition());
    SmartDashboard.putNumber("rotations", (m_flywheel.getPosition() - m_initialPosition)/Config.kticksPerRevolution);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(60);
  }
}
