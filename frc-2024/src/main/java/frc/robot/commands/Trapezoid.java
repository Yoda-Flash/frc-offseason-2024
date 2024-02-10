// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

public class Trapezoid extends Command {
  private Wrist m_wrist;
  private double m_time;
  private double m_speed;
  private double m_accelerationTime;
  private Timer  m_timer  = new Timer();
  private double kP = 0.1;
  /** Creates a new Trapezoid. */
  public Trapezoid(Wrist wrist, double accelerationTime, double speed, double time) {
    m_wrist = wrist;
    m_accelerationTime = accelerationTime;
    m_speed = speed;
    m_time = time;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_wrist.setRotations(0.0);
  }

  private double getTheoreticalPosition(double t){
    if(t < m_accelerationTime){
      return 0.5*(5600/60)*m_speed*(t*t/m_accelerationTime);
    }
    if(t > m_accelerationTime && t < m_time - m_accelerationTime){
      return (5600/60)*m_speed*(t - m_accelerationTime/2);
    }
    return (5600/60)*m_speed*(m_time - m_accelerationTime - 0.5*((m_time-t)*(m_time-t)/m_accelerationTime));
  }
  private double motorSpeed(double t){
    if(t < m_accelerationTime){
      return m_speed*t/m_accelerationTime;
    }
    if(t > m_accelerationTime && t < m_time - m_accelerationTime){
      return m_speed;
    }
    return m_speed*(m_time-t)/m_accelerationTime;
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double theoreticalPosition = getTheoreticalPosition(m_timer.get());
    double error = theoreticalPosition - m_wrist.getRotations();
    m_wrist.setSpeed(motorSpeed(m_timer.get()));
    SmartDashboard.putNumber("error", error);
    SmartDashboard.putNumber("theoretical path", theoreticalPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.setSpeed(0);
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_time);
  }
}
