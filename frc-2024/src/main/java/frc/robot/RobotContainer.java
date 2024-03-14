// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.OperateLED;
import frc.robot.commands.Suibian;
import frc.robot.subsystems.LED;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private static final class Config{
    public static final int kJoystick = 0;
    public static final int kPracticeButtonID = 1;
  }
  
  private LED m_led = new LED();

  private Joystick m_joystick = new Joystick(Config.kJoystick);
  private JoystickButton m_practiceJoystickButton = new JoystickButton(m_joystick, Config.kPracticeButtonID);

  private OperateLED m_operateLED = new OperateLED(m_led);
  private Suibian m_suibian = new Suibian();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {    
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
    m_practiceJoystickButton.whileTrue(m_suibian);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //run rainbow() in a parallelCommand or something
    // An example command will be run in autonomous
    return null;
  }

  public Command getTeleopCommand(){
    m_led.setDefaultCommand(m_operateLED);
    return null;
  }
}
