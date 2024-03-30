// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.LED_State;

public class OperateLED extends Command {
  private LED m_led;
  private Intake m_intake;
  /** Creates a new OperateLED. */
  public OperateLED(LED led, Intake intake) {
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
    if(Math.abs(SmartDashboard.getNumber("Vision/Angle", 0)) <= 0.01) {
      LED.setState(LED_State.RAINBOW);
    }
    else {
      m_intake.setRobotState();
    }
    switch(LED.m_state) {
      case PIECE_STORED:
        m_led.blinking(5); //blinking green
        break;
      case NORMAL:
        m_led.setColor(3); //red
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
