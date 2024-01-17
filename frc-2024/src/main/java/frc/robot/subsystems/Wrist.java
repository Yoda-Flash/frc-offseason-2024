// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
<<<<<<< HEAD
import frc.robot.Constants.WristConstants;
=======
import frc.robot.Constants;
>>>>>>> ff3eb30 (Added wrist subsystem and command)

import com.ctre.phoenix6.hardware.TalonFX;


public class Wrist extends SubsystemBase {

<<<<<<< HEAD
  private TalonFX m_falcon = new TalonFX(WristConstants.kMotorID);
=======
  private TalonFX m_falcon = new TalonFX(Constants.Wrist.kMotorID);
>>>>>>> ff3eb30 (Added wrist subsystem and command)

  public Wrist() {
    m_falcon.setPosition(0);
  }

  public double getEncoderPosition(){
<<<<<<< HEAD
    return m_falcon.getPosition().getValue();
=======
    //return m_falcon.getValue();
    return 0;
>>>>>>> ff3eb30 (Added wrist subsystem and command)
  }

  public void setSpeed(double speed){
    m_falcon.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

