// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimitSwitch extends SubsystemBase {

  private DigitalInput m_limitSwitch;

  public LimitSwitch(int slotDIO) {
    m_limitSwitch = new DigitalInput(slotDIO);
  }

  public boolean limitSwitchTriggered() {
    return m_limitSwitch.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("current digital input: ", limitSwitchTriggered());
    // This method will be called once per scheduler run
  }
}
