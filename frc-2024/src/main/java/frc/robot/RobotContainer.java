// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Commands.BlinkingOrange;
import frc.robot.Commands.Green;
import frc.robot.Commands.MovingOrange;
import frc.robot.Commands.Purple;
import frc.robot.Commands.BlinkingOrange;
import frc.robot.Commands.Green;
import frc.robot.Commands.TurnStillOrange;
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
    public static final int korangeButtonID = 1;
    public static final int kBlinkingOrangeButtonID = 2;
    public static final int kGreenButtonID = 3;
  }

  private LED m_led = new LED();
  //private Joystick m_joystick = new Joystick(Config.kJoystick);

  private TurnStillOrange m_orange = new TurnStillOrange(m_led);
  //private JoystickButton m_orangeButton = new JoystickButton(m_joystick, Config.korangeButtonID);
  
  private MovingOrange m_movingOrange = new MovingOrange(m_led);

  private BlinkingOrange m_blinkingOrange = new BlinkingOrange(m_led);
  //private JoystickButton m_blinkingOrangeButton = new JoystickButton(m_joystick, Config.kBlinkingOrangeButtonID);

  private Green m_green = new Green(m_led);
  //private JoystickButton m_greenButton = new JoystickButton(m_joystick, Config.kGreenButtonID);

  private Purple m_purple = new Purple(m_led);

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
    //m_orangeButton.onTrue(m_orange);
    //m_blinkingOrangeButton.onTrue(m_blinkingOrange);
    //m_greenButton.onTrue(m_green);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_blinkingOrange;
  }
}
