// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.elevator.ArcadeElevator;
import frc.robot.commands.elevator.PIDDown;
import frc.robot.commands.elevator.PIDUp;
import frc.robot.commands.pivot.ArcadePivot;
import frc.robot.commands.swerve.AutoStraighten;
import frc.robot.commands.swerve.JoystickDrive;
import frc.robot.commands.swerve.SnapToAngle;
import frc.robot.commands.wrist.ArcadeWrist;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LimitSwitch;
import frc.robot.subsystems.Pivot;

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
  // private final SwerveDrive m_swerve = new SwerveDrive();

  private final Joystick m_driverJoystick = new Joystick(DriveConstants.kDriveJoystickId);

  // private final JoystickDrive m_drive = new JoystickDrive(m_swerve, 
  //   () -> -m_driverJoystick.getRawAxis(DriveConstants.kJoystickXAxis),
  //   () -> -m_driverJoystick.getRawAxis(DriveConstants.kJoystickYxis),
  //   () -> -m_driverJoystick.getRawAxis(DriveConstants.kJoystickRotAxis)
  // );

  // private SnapToAngle m_snap = new SnapToAngle(m_swerve);
  // private AutoStraighten m_straighten = new AutoStraighten(m_swerve);

  // private JoystickButton m_snapButton = new JoystickButton(m_driverJoystick, Config.kSnapButtonID); 
  // private JoystickButton m_straightenButton = new JoystickButton(m_driverJoystick, Config.kStraightenButtonID);
  // The robot's subsystems and commands are defined here...

  private Pivot m_pivot = new Pivot();
  // private TestPivot m_testPivot = new TestPivot(m_pivot, m_driverJoystick, m_switch1, m_switch2);
  private ArcadePivot m_arcadePivot = new ArcadePivot(m_pivot, m_driverJoystick);

  private Elevator m_elevator = new Elevator();
  private ArcadeElevator m_arcadeElevator = new ArcadeElevator(m_elevator, m_driverJoystick);
  private PIDUp m_elevatorUp = new PIDUp(m_elevator);
  private JoystickButton m_elevatorUpButton = new JoystickButton(m_driverJoystick, 1);
  private PIDDown m_elevatorDown = new PIDDown(m_elevator);
  private JoystickButton m_elevatorDownButton = new JoystickButton(m_driverJoystick, 2);

  private Wrist m_wrist = new Wrist();
  private ArcadeWrist m_arcadeWrist = new ArcadeWrist(m_wrist, m_driverJoystick);

  // Replace with CommandPS4Controller or CommandJoystick if needed


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {    
    // m_swerve.setDefaultCommand(m_drive);

    // Configure the trigger bindings

    configureBindings();
    // SmartDashboard.putData("Swerve/Odo/Reset_Odo", new InstantCommand(() -> m_swerve.resetOdoToPose()));
    // SmartDashboard.putData("Swerve/Odo/Reset_Heading", new InstantCommand(() -> m_swerve.resetHeading()));
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
    // m_snapButton.onTrue(m_snap);
    // m_straightenButton.whileTrue(m_straighten);
    m_elevatorUpButton.whileTrue(m_elevatorUp);
    m_elevatorDownButton.whileTrue(m_elevatorDown);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  public Command getTeleopCommand(){
    // m_arcadePivot.schedule();
    // m_elevator.setDefaultCommand(m_arcadeElevator);
    return m_arcadeWrist;
  }
}
