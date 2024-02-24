// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import java.lang.reflect.Array;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

  private static final class Config {
    public static final int kPort = 0;
    public static final int kLength = 30;
  }

  private AddressableLED m_led = new AddressableLED(Config.kPort);
  private AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(Config.kLength);
  private Timer m_timer = new Timer();

  public enum LED_State {
    ROBOT_MOVING,
    ROBOT_STILL,
    TRYING_PICKUP,
    PIECE_STORED,
    SHOOTING,
    ALIGN,
    RAINBOW /*If I can figure it out */
  }
  private LED_State m_state;

  //private ArrayList<ArrayList<Integer>> m_colorsList = new ArrayList<ArrayList<Integer>>();
  private int [][] m_colorList = {{255,64,0},{0,255,0},{255,0,0},{173,8,191},{0,0,0}};


  /** Creates a new LED. */
  public LED() {
    m_led.setLength(m_buffer.getLength());
  }

  public void setColor(int colorEnum){
    for (var i = 0; i < m_buffer.getLength(); i++) {
      m_buffer.setRGB(i, m_colorList[colorEnum][0], m_colorList[colorEnum][1], m_colorList[colorEnum][2]);
    }
    System.out.println("Run run run");
    m_led.setData(m_buffer);
  }

  public void initLED(){
    m_led.start();
  }

  public void stopLED(){
    m_led.stop();
  }
  public void setIndividualColor(int index, int red, int green, int blue) {
    m_buffer.setRGB(index, red, green, blue);
    m_led.setData(m_buffer);
  }
  public void setState(LED_State newState) {
    m_state = newState;
  }

  public void blinkingOrange() {
    setColor(0);
    m_timer.restart();
    if(m_timer.hasElapsed(0.5) && !m_timer.hasElapsed(1)){
      setColor(4);
    }
    if(m_timer.hasElapsed(1)) {
      setColor(0);
    }
  }

  public void movingOrange() {
    setColor(0);
    m_timer.restart();
    for(var i = 0; i < m_buffer.getLength(); i++){
      if(m_timer.hasElapsed(((0.0 + i)/10)) && !m_timer.hasElapsed(((1.0 + i)/10))){
        setIndividualColor(i, 0, 0, 0);
      }
      if(m_timer.hasElapsed(((1.0 + i)/10))){
        setIndividualColor(i, 255, 64, 0);
      }
    }
  }

  @Override 
  public void periodic() {
    //not sure if this works yet.
    m_led.start();
    // This method will be called once per scheduler run
    switch(m_state) {
      case ROBOT_MOVING:
        break;
      case ROBOT_STILL:
        setColor(2);
        break;
      case TRYING_PICKUP:
        blinkingOrange();
        break;
      case PIECE_STORED:
        setColor(0);
        break;
      case SHOOTING:
        movingOrange();
        break;
      case ALIGN:
        setColor(3);
        break;
      case RAINBOW:
        break;
      default:
        setColor(1);
    }
  }
}
