// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Wrist;

public class TestCommand extends SequentialCommandGroup {
  /** Creates a new TestCommand. */
  public TestCommand(Wrist wrist) {
    addCommands(
      new Move(wrist, 0.5, 3),
      new Move(wrist, 0.7, 2),
      new Move(wrist, 0.2, 4)
    );
  }
}
