// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {

  private TalonFX m_neo1 = new TalonFX(PivotConstants.kMotorID1);
  private TalonFX m_neo2 = new TalonFX(PivotConstants.kMotorID2);
  private TalonFX m_neo3 = new TalonFX(PivotConstants.kMotorID3);
  private TalonFX m_neo4 = new TalonFX(PivotConstants.kMotorID4);
  private DutyCycleEncoder m_encoder = new DutyCycleEncoder(0);


  /** Creates a new ElevatorPivot. */
  public Pivot() {
  //   m_neo2.follow(m_neo1);
  //   m_neo4.follow(m_neo3);
    m_neo1.setInverted(true);
    m_neo2.setInverted(true);

    // m_neo1.getEncoder().setPosition(0);
  }

  // public double getEncoderPosition(){
  //   return m_neo1.getEncoder().getPosition();
  // }

  public void setSpeed(double speed){
    m_neo1.set(speed);
    m_neo3.set(speed);
  }

  public void setLeftSpeed(double speed){
    m_neo3.set(speed);
  }

  public void setRightSpeed(double speed){
    m_neo1.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Absolute Distance", m_encoder.getAbsolutePosition()*360);
  }
}
