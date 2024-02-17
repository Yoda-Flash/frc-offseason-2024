// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Subsystems.LimitSwitch;
import edu.wpi.first.wpilibj.Joystick;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AutoStraighten;
import frc.robot.commands.JoystickDrive;
import frc.robot.commands.SnapToAngle;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private static final class Config{
    public static final int kSnapButtonID = 1;
    public static final int kStraightenButtonID = 2;
  }
  
  // The robot's subsystems and commands are defined here...
  private static final class Config{ // all need to be changed

  }
  private LimitSwitch m_limitSwitch = new LimitSwitch(12);
  private final SwerveDrive m_swerve = new SwerveDrive();

  private final Joystick m_driverJoystick = new Joystick(DriveConstants.kDriveJoystickId);

  private final JoystickDrive m_drive = new JoystickDrive(m_swerve, 
    () -> -m_driverJoystick.getRawAxis(DriveConstants.kJoystickXAxis),
    () -> -m_driverJoystick.getRawAxis(DriveConstants.kJoystickYxis),
    () -> -m_driverJoystick.getRawAxis(DriveConstants.kJoystickRotAxis)
  );

  private SnapToAngle m_snap = new SnapToAngle(m_swerve);
  private AutoStraighten m_straighten = new AutoStraighten(m_swerve);

  private JoystickButton m_snapButton = new JoystickButton(m_driverJoystick, Config.kSnapButtonID); 
  private JoystickButton m_straightenButton = new JoystickButton(m_driverJoystick, Config.kStraightenButtonID);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {    
    m_swerve.setDefaultCommand(m_drive);

    // Configure the trigger bindings

    configureBindings();
    SmartDashboard.putData("Swerve/Odo/Reset_Odo", new InstantCommand(() -> m_swerve.resetOdoToPose()));
    SmartDashboard.putData("Swerve/Odo/Reset_Heading", new InstantCommand(() -> m_swerve.resetHeading()));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_snapButton.onTrue(m_snap);
    m_straightenButton.whileTrue(m_straighten);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("TestAuto");
  }

  public Command getInitCommand(){
    return m_swerve.resetHeadingCommand();
  }
  public Command getTeleopCommand() {
    return null;
  }
}
