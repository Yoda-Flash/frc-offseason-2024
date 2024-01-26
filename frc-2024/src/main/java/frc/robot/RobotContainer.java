// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TurnMovingOrange;
import frc.robot.commands.TurnStillGreen;
import frc.robot.commands.TurnStillOrange;
import frc.robot.subsystem.LED;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private static final class Config{
    public static final int kJoystick = 0;
    public static final int kStillOrangeButtonID = 1;
    public static final int kMovingOrangeButtonID = 2;
    public static final int kStillGreenButtonID = 3;
  }

  private LED m_led = new LED();
  private Joystick m_joystick = new Joystick(Config.kJoystick);

  private TurnStillOrange m_stillOrange = new TurnStillOrange(m_led);
  private JoystickButton m_stillOrangeButton = new JoystickButton(m_joystick, Config.kStillOrangeButtonID);

  private TurnMovingOrange m_movingOrange = new TurnMovingOrange(m_led);
  private JoystickButton m_movingOrangeButton = new JoystickButton(m_joystick, Config.kMovingOrangeButtonID);

  private TurnStillGreen m_stillGreen = new TurnStillGreen(m_led);
  private JoystickButton m_stillGreenButton = new JoystickButton(m_joystick, Config.kStillGreenButtonID);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
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
    m_stillOrangeButton.onTrue(m_stillOrange);
    m_movingOrangeButton.onTrue(m_movingOrange);
    m_stillGreenButton.onTrue(m_stillGreen);
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
}
