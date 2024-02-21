// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
import frc.robot.subsystems.LimitSwitch;
import edu.wpi.first.wpilibj.Joystick;
>>>>>>> 89f08a7 (Tested pivot, need to update Falcons to v6)
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
=======
>>>>>>> cf483b6 (Made all the commands/subsystems for the elevator pivot)
=======
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
>>>>>>> 3d2e4b4 (Got pivot working with arcade)
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
<<<<<<< HEAD
import frc.robot.commands.AutoStraighten;
import frc.robot.commands.JoystickDrive;
import frc.robot.commands.SnapToAngle;
import frc.robot.subsystems.LimitSwitch;
=======
import frc.robot.commands.pivot.ArcadePivot;
import frc.robot.commands.pivot.TestPivot;
import frc.robot.commands.swerve.AutoStraighten;
import frc.robot.commands.swerve.JoystickDrive;
import frc.robot.commands.swerve.SnapToAngle;
>>>>>>> 89f08a7 (Tested pivot, need to update Falcons to v6)
import frc.robot.subsystems.SwerveDrive;
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
  
<<<<<<< HEAD
  private LimitSwitch m_limitSwitch = new LimitSwitch(12);
  private final SwerveDrive m_swerve = new SwerveDrive();
=======
  // The robot's subsystems and commands are defined here...
  // private final SwerveDrive m_swerve = new SwerveDrive();
>>>>>>> 89f08a7 (Tested pivot, need to update Falcons to v6)

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

  // Replace with CommandPS4Controller or CommandJoystick if needed

<<<<<<< HEAD
=======
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
>>>>>>> cf483b6 (Made all the commands/subsystems for the elevator pivot)
=======
>>>>>>> 3d2e4b4 (Got pivot working with arcade)

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
<<<<<<< HEAD
=======
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`


    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
>>>>>>> cf483b6 (Made all the commands/subsystems for the elevator pivot)
=======
>>>>>>> 3d2e4b4 (Got pivot working with arcade)
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
<<<<<<< HEAD
>>>>>>> cf483b6 (Made all the commands/subsystems for the elevator pivot)
=======
  }

  public Command getTeleopCommand(){
    // m_arcadePivot.schedule();
    return m_arcadePivot;
>>>>>>> 3d2e4b4 (Got pivot working with arcade)
  }
}
