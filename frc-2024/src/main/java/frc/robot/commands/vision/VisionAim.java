// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.PositionConstants;
import frc.robot.commands.pivot.TrapezoidalPivot;
import frc.robot.commands.wrist.TrapezoidalWrist;
import frc.robot.commands.wrist.TrapezoidalWristProvider;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class VisionAim extends ParallelCommandGroup {
  /** Creates a new VisionAim. */
  public VisionAim(Wrist wrist, Pivot pivot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    //  new VisionDistanceToAngle(swerve),
     new TrapezoidalWristProvider(wrist, () -> wrist.getAutoAimSetpoint()),
     new TrapezoidalPivot(pivot, PositionConstants.kSubwooferPivot)
    );
  }
}
