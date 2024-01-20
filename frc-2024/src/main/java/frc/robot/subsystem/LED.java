// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

  private static final class Config {
    public static final int kPort = 0;
    public static final int kLength = 30;
  }

  private AddressableLED m_led = new AddressableLED(Config.kPort);
  private AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(Config.kLength);

  /** Creates a new LED. */
  public LED() {
    m_led.setLength(m_buffer.getLength());
  }

  public void setOrange(){
    for (var i = 0; i < m_buffer.getLength(); i++) {
      m_buffer.setRGB(i, 246, 144, 8);
    }

    m_led.setData(m_buffer);
  }

  public void initLED(){
    m_led.start();
  }

  public InstantCommand turnOrange(){
    return new InstantCommand(this::setOrange, this);
  }

  @Override 
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
