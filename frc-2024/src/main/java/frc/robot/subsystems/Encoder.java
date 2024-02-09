// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder; // Absolute Encoder might be deprecated later
// see https://codedocs.revrobotics.com/java/com/revrobotics/package-summary for commands/documentation

import edu.wpi.first.wpilibj.DutyCycleEncoder;



public class Encoder extends SubsystemBase {
  /** Creates a new Encoder. */

  DutyCycleEncoder m_encoder;

  public Encoder() {
    m_encoder = new DutyCycleEncoder(9);  
  }


  public double get_position() {
    return m_encoder.getAbsolutePosition();
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
