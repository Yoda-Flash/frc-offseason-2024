// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  private static final class Config {
    public static final int kPort = 0;
    public static final int kLength = 30; //change later
  }

  private AddressableLED m_led = new AddressableLED(Config.kPort);
  private AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(Config.kLength);

  public enum LED_State {
    TRYING_PICKUP,
    PIECE_STORED,
    SHOOTING,
    NORMAL,
    RAINBOW /*If I can figure it out */
  }
  public static LED_State m_state;

  private int [][] m_colorList = {{0,0,0},{255,64,0},{0,0,255},{255,0,0},{173,8,191}};
  private int m_rainbowFirstPixelHue;

  /** Creates a new LED. */
  public LED() {
    m_led.setLength(m_buffer.getLength());
    m_led.start(); //maybe comment out
    m_rainbowFirstPixelHue = 0;
    setState(LED_State.RAINBOW);
  }

  public void setColor(int color){
    for (var i = 0; i < m_buffer.getLength(); i++) {
      m_buffer.setRGB(i, m_colorList[color][0], m_colorList[color][1], m_colorList[color][2]);
    }
    System.out.println("setColor");
    m_led.setData(m_buffer);
  }

  public void setIndividualColor(int index, int red, int green, int blue) {
    m_buffer.setRGB(index, red, green, blue);
    m_led.setData(m_buffer);
  }

  public void setIndividualColor(int index, int color) {
    m_buffer.setRGB(index, m_colorList[color][0], m_colorList[color][1], m_colorList[color][2]);
    m_led.setData(m_buffer);
  }

  public static void setState(LED_State newState) {
    m_state = newState;
  }

  public void rainbow() {
    for (var i = 0; i < m_buffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_buffer.getLength())) % 180;
      // Set the value
      m_buffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
    m_led.setData(m_buffer);
  }

  public void initLED(){
    m_led.start();
  }

  public void stopLED(){
    m_led.stop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
