// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.LED_State;

public class OperateLED extends Command {
  private LED m_led;
  /** Creates a new OperateLED. */
  public OperateLED(LED led) {
    m_led = led;
    addRequirements(m_led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(LED.m_state) {
      case TRYING_PICKUP:
        m_led.setColor(1);
        break;
      case PIECE_STORED:
        m_led.setColor(2);
        break;
      case SHOOTING:
        break;
      case ALIGN:
        break;
      case RAINBOW:
        m_led.rainbow();
        break;
      default:
        m_led.setColor(3);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
