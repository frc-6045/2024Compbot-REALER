// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  /** Creates a new LEDs. */
  private AddressableLED m_LED;
  private AddressableLEDBuffer m_LEDBuffer;
  public LEDs() {
    m_LED = new AddressableLED(9);
    m_LEDBuffer = new AddressableLEDBuffer(39);
    m_LED.setLength(m_LEDBuffer.getLength());

    m_LED.setData(m_LEDBuffer);
    m_LED.start();
    for(int i = 0; i < m_LEDBuffer.getLength(); i++){
      m_LEDBuffer.setRGB(i, 39, 2, 201);
    }

    m_LED.setData(m_LEDBuffer);
    System.out.println("led on");

  }

  public void setColor(int red, int green, int blue){
    for(int i = 0; i < m_LEDBuffer.getLength(); i++){
      m_LEDBuffer.setRGB(i, red, green, blue);
    }
    m_LED.setData(m_LEDBuffer);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
