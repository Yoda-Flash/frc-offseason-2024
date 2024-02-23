// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.LED;

public class Purple extends Command {
  private LED m_led;
  /** Creates a new Purple. */
  public Purple(LED led) {
    m_led = led;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_led.initLED();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_led.setColor(3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_led.stopLED();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
