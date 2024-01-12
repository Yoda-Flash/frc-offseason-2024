// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorPivot;
import frc.robot.Constants;

public class RotatePivot extends Command {

  private double m_endPosition;
  private ElevatorPivot m_elevatorPivot;
  private PIDController m_PID = new PIDController(Constants.ElevatorPivot.elevatorPivot_kP, Constants.ElevatorPivot.elevatorPivot_kI, Constants.ElevatorPivot.elevatorPivot_kD);
  /** Creates a new RotatePivot. */
  public RotatePivot(double a, ElevatorPivot elevatorPivot) {
    m_elevatorPivot = elevatorPivot;
    m_endPosition = (90 - a)*Constants.ElevatorPivot.kNeoTicksPerRevolution/360;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevatorPivot.setSpeed(m_PID.calculate(m_elevatorPivot.getEncoderPosition(), m_endPosition));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevatorPivot.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_elevatorPivot.getEncoderPosition() - m_endPosition) < Constants.ElevatorPivot.kDeadBand);
  }
}
