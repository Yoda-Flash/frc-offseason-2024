// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Motor;

import static edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class TrapezoidProfileTest extends Command {

  private Motor m_motor;
  private TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(4000, 500));
  private PIDController m_pid = new PIDController(0, 0, 0);
  private SimpleMotorFeedforward m_ff = new SimpleMotorFeedforward(0, 0.000173267300556);
  private Timer m_timer = new Timer();
  private State m_setpoint = new State();

  /** Creates a new TrapezoidProfileTest. */
  public TrapezoidProfileTest(Motor motor) {
    m_motor = motor;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("I'm running!");
    m_timer.restart();
    m_motor.setRotations(0);
    m_setpoint = new State();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_setpoint = m_profile.calculate(0.02, new State(m_motor.getRotations(), m_motor.getVelocity()), 
      new State(1000, 0.0));
    // m_motor.setSpeed(m_pid.calculate(m_motor.getRotations(), m_setpoint.position));
    double speed = m_ff.calculate(m_setpoint.velocity);
    m_motor.setSpeed(speed);

    SmartDashboard.putNumber("FFTest/Trapezoid", speed);
    SmartDashboard.putNumber("FFTest/RPMTrap", m_setpoint.velocity);
    SmartDashboard.putNumber("FFTest/position", m_motor.getRotations());
    SmartDashboard.putNumber("FFTest/velocity", m_motor.getVelocity());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("stopping!");
    m_motor.setSpeed(0);
    m_motor.setRotations(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return m_timer.get() > 5.0;
  }
}
