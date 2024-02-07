// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  
  private TalonFX m_falcon = new TalonFX(0);

  public Wrist() {

  }

  public void setSpeed(double speed){
    m_falcon.set(speed);
  }

  public double getRotations(){
    return m_falcon.getPosition().getValue();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
