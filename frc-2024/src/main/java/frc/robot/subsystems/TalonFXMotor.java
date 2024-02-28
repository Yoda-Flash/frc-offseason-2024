// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// 13, 60, 18, 30
// 7.6923

/*
 * Position: Rotations
 * Velocity: Rotations per second (not minute)
 */
public class TalonFXMotor extends SubsystemBase {

  private TalonFX m_falcon = new TalonFX(4);
  private StatusSignal<Double> m_position = m_falcon.getPosition();
  private StatusSignal<Double> m_velocity = m_falcon.getVelocity();

  /** Creates a new Motor. */
  public TalonFXMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.SensorToMechanismRatio = 9.0909;
    config.Feedback.RotorToSensorRatio = 1.0;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    m_falcon.getConfigurator().apply(config);
  }

  public void setSpeed(double speed){
    m_falcon.set(speed);
  }

  public double getRotations() {
    return m_position.refresh().getValueAsDouble();
  }

  public void setRotations(double r){
    m_falcon.setPosition(r);
  }

  public void setVoltage(double r){
    m_falcon.setVoltage(-r);
  }

  public double getVelocity() {
    return m_velocity.refresh().getValueAsDouble();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("FFTest/Distance", getRotations());
    SmartDashboard.putNumber("Velocity", m_falcon.get());
  }
}
