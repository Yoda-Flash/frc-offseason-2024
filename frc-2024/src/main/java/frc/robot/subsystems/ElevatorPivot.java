// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;

public class ElevatorPivot extends SubsystemBase {

  private CANSparkMax m_neo1 = new CANSparkMax(Constants.PivotConstants.kMotorID1, MotorType.kBrushless);
  private CANSparkMax m_neo2 = new CANSparkMax(Constants.PivotConstants.kMotorID2, MotorType.kBrushless);
  private CANSparkMax m_neo3 = new CANSparkMax(Constants.PivotConstants.kMotorID3, MotorType.kBrushless);
  private CANSparkMax m_neo4 = new CANSparkMax(Constants.PivotConstants.kMotorID4, MotorType.kBrushless);
  private DutyCycleEncoder m_encoder = new DutyCycleEncoder(0);


  /** Creates a new ElevatorPivot. */
  public ElevatorPivot() {
    m_neo3.follow(m_neo1);
    m_neo4.follow(m_neo2);
    m_neo2.setInverted(true);
    m_neo4.setInverted(true);

    m_neo1.getEncoder().setPosition(0);
  }

  public double getEncoderPosition(){
    return m_neo1.getEncoder().getPosition();
  }

  public void setSpeed(double speed){
    m_neo1.set(speed);
    m_neo2.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Absolute Distance", m_encoder.getDistance());
  }
}
