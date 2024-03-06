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
import frc.robot.commands.elevator.ElevatorRecalibrate;
import frc.robot.commands.elevator.PIDDown;
import frc.robot.commands.elevator.PIDUp;
import frc.robot.commands.groups.AmpScore;
import frc.robot.commands.groups.BackwardIntake;
import frc.robot.commands.groups.Climb;
import frc.robot.commands.groups.ForwardIntake;
import frc.robot.commands.groups.GroundIntake;
import frc.robot.commands.groups.SpeakerScore;
import frc.robot.commands.groups.Stowed;
import frc.robot.commands.intakeshooter.ArcadeIntake;
import frc.robot.commands.intakeshooter.ArcadeShoot;
import frc.robot.commands.intakeshooter.Outtake;
import frc.robot.commands.intakeshooter.ReverseShooter;
import frc.robot.commands.pivot.ArcadePivot;
import frc.robot.commands.pivot.PIDBack;
import frc.robot.commands.pivot.PIDFront;
import frc.robot.commands.pivot.PivotRecalibrate;
import frc.robot.commands.swerve.AutoStraighten;
import frc.robot.commands.swerve.JoystickDrive;
import frc.robot.commands.swerve.SnapToAngle;
import frc.robot.commands.swerve.VisionSnapToAngle;
import frc.robot.commands.wrist.ArcadeWrist;
import frc.robot.commands.wrist.PIDDrop;
import frc.robot.commands.wrist.PIDRaise;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Wrist;

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
    public static final int kElevatorUpButtonID = 1;
    public static final int kElevatorDownButtonID = 2;
    public static final int kPivotForwardButtonID = 3;
    public static final int kPivotBackwardButtonID = 4;
    public static final int kWristForwardButtonID = 5;
    public static final int kWristBackwardButtonID = 6;
    public static final int kIntakeButtonID = 7;
    public static final int kShooterButtonID = 8;
  }
  
  private final SwerveDrive m_swerve = new SwerveDrive();

  private final Joystick m_driverJoystick = new Joystick(DriveConstants.kDriveJoystickId);
  private final Joystick m_joystick2 = new Joystick(Constants.kJoystick2ID);

  private final JoystickDrive m_drive = new JoystickDrive(m_swerve, 
    () -> -m_driverJoystick.getRawAxis(DriveConstants.kJoystickXAxis),
    () -> -m_driverJoystick.getRawAxis(DriveConstants.kJoystickYxis),
    () -> -m_driverJoystick.getRawAxis(DriveConstants.kJoystickRotAxis)
  );

  private SnapToAngle m_snap = new SnapToAngle(m_swerve);
  private AutoStraighten m_straighten = new AutoStraighten(m_swerve);
  private VisionSnapToAngle m_visionSnap = new VisionSnapToAngle(m_swerve);

  private JoystickButton m_snapButton = new JoystickButton(m_driverJoystick, Config.kSnapButtonID); 
  private JoystickButton m_straightenButton = new JoystickButton(m_driverJoystick, Config.kStraightenButtonID);
  // The robot's subsystems and commands are defined here...

  private Pivot m_pivot = new Pivot();
  private ArcadePivot m_arcadePivot = new ArcadePivot(m_pivot, m_joystick2);
  private PIDFront m_pivotForward = new PIDFront(m_pivot);
  private JoystickButton m_pivotFowardButton = new JoystickButton(m_joystick2, Config.kPivotForwardButtonID);
  private PIDBack m_pivotBackward = new PIDBack(m_pivot);
  private JoystickButton m_pivotBackwardButton = new JoystickButton(m_joystick2, Config.kPivotBackwardButtonID);
  private PivotRecalibrate m_pivotRecalibrate = new PivotRecalibrate(m_pivot);

  private Elevator m_elevator = new Elevator();
  private ArcadeElevator m_arcadeElevator = new ArcadeElevator(m_elevator, m_joystick2);
  private PIDUp m_elevatorUp = new PIDUp(m_elevator);
  private JoystickButton m_elevatorUpButton = new JoystickButton(m_joystick2, Config.kElevatorUpButtonID);
  private PIDDown m_elevatorDown = new PIDDown(m_elevator);
  private JoystickButton m_elevatorDownButton = new JoystickButton(m_joystick2, Config.kElevatorDownButtonID);
  private ElevatorRecalibrate m_elevatorRecalibrate = new ElevatorRecalibrate(m_elevator);

  private Wrist m_wrist = new Wrist();
  private ArcadeWrist m_arcadeWrist = new ArcadeWrist(m_wrist, m_joystick2);
  private PIDRaise m_wristForward = new PIDRaise(m_wrist);
  private JoystickButton m_wristForwardButton = new JoystickButton(m_joystick2, Config.kWristForwardButtonID);
  private PIDDrop m_wristBackward = new PIDDrop(m_wrist);
  private JoystickButton m_wristBackwardButton = new JoystickButton(m_joystick2, Config.kWristBackwardButtonID);

  private Intake m_intake = new Intake();
  private ArcadeIntake m_runIntake = new ArcadeIntake(m_intake, m_joystick2);
  private Outtake m_outtake = new Outtake(m_intake, -0.6);
  private JoystickButton m_intakeButton = new JoystickButton(m_joystick2, Config.kIntakeButtonID);

  private Shooter m_shooter = new Shooter();
  private ArcadeShoot m_shoot = new ArcadeShoot(m_shooter, m_joystick2);
  private ReverseShooter m_reverseShooter = new ReverseShooter(m_shooter, -0.5);
  private JoystickButton m_shooterButton = new JoystickButton(m_joystick2, Config.kShooterButtonID);

  private ForwardIntake m_forwardIntake = new ForwardIntake(m_pivot, m_wrist);
  private BackwardIntake m_backwardIntake = new BackwardIntake(m_pivot, m_wrist);
  private AmpScore m_ampScore = new AmpScore(m_pivot, m_wrist, m_elevator);
  private Climb m_climb = new Climb(m_pivot, m_wrist, m_elevator);
  private GroundIntake m_groundIntake = new GroundIntake(m_pivot, m_wrist, m_elevator);
  private SpeakerScore m_speakerScore = new SpeakerScore(m_pivot, m_wrist, m_elevator);
  private Stowed m_stowed = new Stowed(m_pivot, m_wrist, m_elevator);

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {    
    // m_swerve.setDefaultCommand(m_drive);

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
    m_snapButton.onTrue(m_visionSnap);
    m_straightenButton.whileTrue(m_straighten);

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`


    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_elevatorUpButton.onTrue(m_stowed);
    m_elevatorDownButton.whileTrue(m_ampScore);
    // m_elevatorUpButton.whileTrue(m_elevatorUp);
    // m_elevatorDownButton.whileTrue(m_elevatorDown);
    m_pivotFowardButton.whileTrue(m_groundIntake);
    // m_pivotBackwardButton.whileTrue(m_pivotRecalibrate);

    // m_wristForwardButton.whileTrue(m_wristForward);
    // m_wristBackwardButton.whileTrue(m_wristBackward);
    // m_elevatorUpButton.whileTrue(m_forwardIntake);
    // m_elevatorDownButton.whileTrue(m_backwardIntake);
    m_intakeButton.whileTrue(m_outtake);
    // m_intakeButton.whileTrue(m_stowed);
    // m_shooterButton.whileTrue(m_ampScore); 
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
    m_pivot.setDefaultCommand(m_arcadePivot);
    m_wrist.setDefaultCommand(m_arcadeWrist);
    m_intake.setDefaultCommand(m_runIntake);
    m_shooter.setDefaultCommand(m_shoot);
    m_elevator.setDefaultCommand(m_arcadeElevator);
    m_swerve.setDefaultCommand(m_drive);
    return null;
  }
}
