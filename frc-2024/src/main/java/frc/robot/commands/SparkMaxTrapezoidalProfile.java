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
import frc.robot.subsystems.SparkMaxMotor;

import static edu.wpi.first.math.trajectory.TrapezoidProfile.State;

import com.revrobotics.SparkMaxAlternateEncoder;

public class SparkMaxTrapezoidalProfile extends Command {

  private SparkMaxMotor m_motor;
  private TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(3000, 3000.0 * 60.0));
  private PIDController m_pid = new PIDController(0.4, 0, 0);
  private SimpleMotorFeedforward m_ff = new SimpleMotorFeedforward(0.0, 0.00017156857726);
  private State m_sample = new State();
  private State m_goal;
  private double m_prevTime;

  /** Creates a new TrapezoidProfileTest. */
  public SparkMaxTrapezoidalProfile(SparkMaxMotor motor, double goal) {
    m_motor = motor;
    m_goal = new State(goal, 0);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_motor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println("I'm running!");
    m_motor.setRotations(0);
    m_sample = new State(0.0, 0.0);
    m_prevTime = Timer.getFPGATimestamp();
    // System.out.println("Init rotations:" + m_motor.getRotations());
    SmartDashboard.putBoolean("FFTest/startstop", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Calculate measured and setpoint states.
    double currTime = Timer.getFPGATimestamp();
    m_sample = m_profile.calculate((currTime - m_prevTime) / 60.0, m_sample, m_goal);
    m_prevTime = currTime;

    double ff = m_ff.calculate(m_sample.velocity);
    double pid = m_pid.calculate(m_motor.getRotations(), m_sample.position);
    double speed = ff + pid;
    m_motor.setSpeed(speed);

    SmartDashboard.putNumber("FFTest/Trapezoid", speed);
    SmartDashboard.putNumber("FFTest/RPMTrap", m_sample.velocity);
    SmartDashboard.putNumber("FFTest/PosTrap", m_sample.position);
    SmartDashboard.putNumber("FFTest/position", m_motor.getRotations());
    SmartDashboard.putNumber("FFTest/velocity", m_motor.getVelocity());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("stopping!");
    m_motor.setSpeed(0);
    m_motor.setRotations(0);
    SmartDashboard.putBoolean("FFTest/startstop", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {  
    double deadband = 0.005;
    return Math.abs(m_motor.getRotations() - m_goal.position) <= deadband;
  }
}
