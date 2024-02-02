// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.LimitSwitch;

public class SwitchSwitching extends Command {
  private LimitSwitch m_limitSwitch;
  /** Creates a new SwitchSwitching. */
  public SwitchSwitching(LimitSwitch limi) {
    m_limitSwitch = limi;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_limitSwitch);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("initial digital input: ", m_limitSwitch.limitSwitchTriggered());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("current digital input: ", m_limitSwitch.limitSwitchTriggered());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
