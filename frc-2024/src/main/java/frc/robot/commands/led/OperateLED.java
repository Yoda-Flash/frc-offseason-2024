// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;

public class OperateLED extends Command {
  private LED m_led;
  /** Creates a new OperateLED. */
  public OperateLED(LED led) {
    m_led = led;
    addRequirements(m_led);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(LED.m_state) {
      case TRYING_PICKUP:
        m_led.setColor(1); //orange - not used
        System.out.println("picking up");
        break;
      case PIECE_STORED:
        m_led.setColor(3); //red
        break;
      case SHOOTING:
        m_led.setColor(4); //purple - not used
        break;
      case NORMAL:
        m_led.setColor(2); //blue
        System.out.println("Back to normal!");
        break;
      case RAINBOW:
        m_led.rainbow();
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
