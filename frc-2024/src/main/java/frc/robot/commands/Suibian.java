// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.LED_State;

public class Suibian extends Command {
  /** Creates a new Suibian. */
  public Suibian() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LED.setState(LED_State.TRYING_PICKUP);
    System.out.println("Suibian!");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(LED.m_state != LED_State.PIECE_STORED) {
      LED.setState(LED_State.NORMAL);
    }
    System.out.println("hello");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
