// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

public class ArcadeWrist extends Command {

  private static final class Config{
    public static final int kAxis = 5;
    public static final double kMultiplier = 0.5;
  }

  private Wrist m_wrist;
  private Joystick m_joystick;
  private double m_joystickInput;

  /** Creates a new ArcadeWrist. */
  public ArcadeWrist(Wrist wrist, Joystick joystick) {
    m_wrist = wrist;
    m_joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist.setSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_joystickInput = m_joystick.getRawAxis(Config.kAxis)*Config.kMultiplier;
    m_wrist.setSpeed(m_joystickInput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
