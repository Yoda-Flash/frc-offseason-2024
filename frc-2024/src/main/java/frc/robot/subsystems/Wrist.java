// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  
  private CANSparkMax m_neo = new CANSparkMax(4, MotorType.kBrushless);

  public Wrist() {
    m_neo.getEncoder().setPosition(0.0);
  }

  public void setSpeed(double speed){
    m_neo.set(speed);
  }

  public double getRotations(){
    return m_neo.getEncoder().getPosition();
  }

  public void setRotations(double r){
    m_neo.getEncoder().setPosition(r);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Distance", getRotations());
    SmartDashboard.putNumber("Velocity", m_neo.get());
  }
}