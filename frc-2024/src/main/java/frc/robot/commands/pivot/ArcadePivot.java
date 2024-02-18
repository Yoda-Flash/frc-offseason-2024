// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimitSwitch;
import frc.robot.subsystems.Pivot;

public class ArcadePivot extends Command {

    private static final class Config{
    public static final int kAxis = 1;
    public static final double kMultiplier = 0.1;
  }

  private Pivot m_pivot;
  private Joystick m_joystick;
  private LimitSwitch m_switch1;
  private LimitSwitch m_switch2;

  /** Creates a new ArcadePivot. */
  public ArcadePivot(Pivot pivot, Joystick joystick, LimitSwitch switch1, LimitSwitch switch2) {
    m_pivot = pivot;
    m_joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("I'm running");
    m_pivot.setSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("I'm executing");
    m_pivot.setSpeed(m_joystick.getRawAxis(Config.kAxis)*Config.kMultiplier);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivot.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  //   if (m_switch1.ifTriggered() || m_switch2.ifTriggered()){
  //   return true;
  // } else {
  //   return false;
  // }
    return false;
  }
}