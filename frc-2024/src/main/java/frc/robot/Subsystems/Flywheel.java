// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {

  private static final class Config {
    public static final int kleftFlywheelMotorID = 1; //change later
    public static final int krightFlywheelMotorID = 2; //change later
    public static final double kflywheelMotorSpeed = 0.5; //change later
  }

  private CANSparkMax m_leftFlywheelMotor = new CANSparkMax(Config.kleftFlywheelMotorID, MotorType.kBrushless);
  private CANSparkMax m_rightFlywheelMotor = new CANSparkMax(Config.krightFlywheelMotorID, MotorType.kBrushless);

  /** Creates a new Flywheel. */
  public Flywheel() {
    m_rightFlywheelMotor.setInverted(true); //change later after testing

    m_leftFlywheelMotor.setSmartCurrentLimit(15, 15); //change later
    m_rightFlywheelMotor.setSmartCurrentLimit(15, 15); //chane later

    m_leftFlywheelMotor.burnFlash();
    m_rightFlywheelMotor.burnFlash();
  }

  public void shoot() {
    m_leftFlywheelMotor.set(Config.kflywheelMotorSpeed);
    m_rightFlywheelMotor.set(Config.kflywheelMotorSpeed);
  }

  public InstantCommand shootNote() {
    return new InstantCommand(this::shoot, this);
  }

  public void zero() {
    m_leftFlywheelMotor.set(0);
    m_rightFlywheelMotor.set(0);
  }

  public InstantCommand stopShooting() {
    return new InstantCommand(this::zero, this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("L Flywheel Motor Temp", m_leftFlywheelMotor.getMotorTemperature());
    SmartDashboard.putNumber("R Flywhell Motor Temp", m_rightFlywheelMotor.getMotorTemperature());

    SmartDashboard.putNumber("L Flywheel Motor Current", m_leftFlywheelMotor.getOutputCurrent());
    SmartDashboard.putNumber("R Flywheel Motor Current", m_rightFlywheelMotor.getOutputCurrent());
  }
}
