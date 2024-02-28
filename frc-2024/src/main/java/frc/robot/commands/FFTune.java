// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Motor;

public class FFTune extends Command {
  private Motor m_motor;
  /** Creates a new FFTune. */
  public FFTune(Motor motor) {
    m_motor = motor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(motor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("FFTest/speed", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double s = SmartDashboard.getNumber("FFTest/speed", 0);
    m_motor.setSpeed(s);


    SmartDashboard.putNumber("FFTest/CalculatedKV", s / m_motor.getVelocity());
    SmartDashboard.putNumber("FFTest/RPM", m_motor.getVelocity());
    SmartDashboard.putNumber("FFTest/Rots", m_motor.getRotations());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_motor.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
