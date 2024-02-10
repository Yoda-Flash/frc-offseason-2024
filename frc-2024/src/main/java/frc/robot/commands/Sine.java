// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Sine extends Command {
  private Wrist m_wrist;
  private double m_time;
  private double m_speed0;
  private double m_frequency;
  private Timer  m_timer  = new Timer();
  private double kP = 0.1;
  /** Creates a new Move. */
  public Sine(Wrist wrist, double speed0, double time, double frequency) {
    m_wrist = wrist;
    m_frequency = frequency;
    m_speed0 = speed0;
    m_time = time;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_wrist.setRotations(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double theoreticalPosition = (5600/60)*m_speed0-(5600/60)*m_speed0*Math.cos((m_timer.get()*m_frequency))/m_frequency;
    double error = theoreticalPosition-(m_wrist.getRotations());
    m_wrist.setSpeed(m_speed0*Math.sin(m_timer.get()*m_frequency) + kP*error);
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
