// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
<<<<<<< HEAD
=======
  private TalonFX m_falcon = new TalonFX(Constants.Wrist.kMotorID);
>>>>>>> ff3eb30 (Added wrist subsystem and command)
=======
>>>>>>> 3d2e4b4 (Got pivot working with arcade)
=======
  private LimitSwitch m_forward = new LimitSwitch(5);
  private LimitSwitch m_backward = new LimitSwitch(4);
  private DutyCycleEncoder m_encoder = new DutyCycleEncoder(6);
>>>>>>> fd0313e (Tuned pivot PID somewhat, need to debug wrist PID)

  public Wrist() {
    m_falcon.setPosition(0);
    m_falcon.setNeutralMode(NeutralModeValue.Brake);
  }

  public double getEncoderPosition(){
<<<<<<< HEAD
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
=======
    return m_encoder.getAbsolutePosition() - WristConstants.kEncoderOffset;
  }

  public boolean ifForwardTriggered(){
    return m_forward.ifTriggered();
  }

  public boolean ifBackwardTriggered(){
    return m_backward.ifTriggered();
>>>>>>> fd0313e (Tuned pivot PID somewhat, need to debug wrist PID)
  }

  public void setSpeed(double speed){
    if (!ifForwardTriggered() && speed<0) {
      speed = 0;
    } else if (!ifBackwardTriggered() && speed>0){
      speed = 0;
    }
    SmartDashboard.putNumber("Wrist speed", speed);
    m_falcon.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Wrist forward", ifForwardTriggered());
    SmartDashboard.putBoolean("Wrist backward", ifBackwardTriggered());
    SmartDashboard.putNumber("Wrist encoder", getEncoderPosition());
  }
}

