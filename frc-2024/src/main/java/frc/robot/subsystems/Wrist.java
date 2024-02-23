// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
<<<<<<< HEAD
=======
import frc.robot.Constants;
>>>>>>> ff3eb30 (Added wrist subsystem and command)

=======
>>>>>>> 3d2e4b4 (Got pivot working with arcade)
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class Wrist extends SubsystemBase {

  private TalonFX m_falcon = new TalonFX(WristConstants.kMotorID);
<<<<<<< HEAD
=======
  private TalonFX m_falcon = new TalonFX(Constants.Wrist.kMotorID);
>>>>>>> ff3eb30 (Added wrist subsystem and command)
=======
>>>>>>> 3d2e4b4 (Got pivot working with arcade)

  public Wrist() {
    m_falcon.setPosition(0);
    m_falcon.setNeutralMode(NeutralModeValue.Brake);
  }

  public double getEncoderPosition(){
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    return m_falcon.getPosition().getValue();
=======
    //return m_falcon.getValue();
=======
    return m_falcon.getSelectedSensorPosition();
>>>>>>> 471ebc4 (Update Wrist.java)
    return 0;
>>>>>>> ff3eb30 (Added wrist subsystem and command)
=======
    return m_falcon.getPosition().getValue();
>>>>>>> 10d91f2 (Fixed falcon problem)
  }

  public void setSpeed(double speed){
    m_falcon.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

