// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.LED;

public class MovingOrange extends Command {
  private LED m_led;
  private Timer m_timer;
  /** Creates a new MovingOrange. */
  public MovingOrange(LED led) {
    m_led = led;
    m_timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_led.initLED();
    m_led.setColor(0);
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    for(var i = 0; i < 30; i++){
      if(m_timer.hasElapsed(((0.0 + i)/10)) && !m_timer.hasElapsed(((1.0 + i)/10))){
        m_led.setIndividualColor(i, 0, 0, 0);
      }
      if(m_timer.hasElapsed(((1.0 + i)/10))){
        m_led.setIndividualColor(i, 255, 64, 0);
      }
    }
    if (m_timer.hasElapsed(3.001)){
      m_timer.restart();
    }
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
