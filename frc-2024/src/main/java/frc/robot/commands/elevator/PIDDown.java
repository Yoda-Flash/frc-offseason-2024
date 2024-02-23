// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class PIDDown extends Command {

  private Elevator m_elevator;
  private PIDController m_pid = new PIDController(0.15, 0, 0);

  /** Creates a new PIDElevatorTest. */
  public PIDDown(Elevator elevator) {
    m_elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var speed = m_pid.calculate(m_elevator.getEncoderPosition(), 0.70);
    System.out.println("I'm running");

    if (!(Math.abs(m_elevator.getEncoderPosition() - 0.70)<= 0.05)){
    System.out.println("I'm running in if-else loop");
    System.out.println(speed);
    SmartDashboard.putNumber("PID value", speed);
    m_elevator.setSpeed(speed);
  
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_elevator.getEncoderPosition() - 0.70)<= 0.05);
  }
}
