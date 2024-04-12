// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LEDSchema;

public class LEDs extends SubsystemBase {
  /** Creates a new LEDs. */
  private AddressableLED m_LED;
  private AddressableLEDBuffer m_LEDBuffer;
  private boolean LEDMode;
  private LEDSchema ledSchema;
  private ArrayList<Runnable> ledSchemes;
  private int schemeIndex;
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
    LEDMode = false;
    schemeIndex = 0;
    // ledSchema = new LEDSchema(this);
    // ledSchemes.add(ledSchema::TransSchema);
    // ledSchemes.add(ledSchema::GaySchema);
  }

  public void setColor(int red, int green, int blue){
    for(int i = 0; i < m_LEDBuffer.getLength(); i++){
      m_LEDBuffer.setRGB(i, red, green, blue);
    }
    m_LED.setData(m_LEDBuffer);
  }

  public void setPixelColor(int index, int red, int green, int blue){
    m_LEDBuffer.setRGB(index, red, green, blue);
    m_LED.setData(m_LEDBuffer);
  }

  public void checkSchemeIndex(){
    if(schemeIndex >= ledSchemes.size()){
      schemeIndex = 0;
    }
  }

  // public void loadCurrentScheme(){
  //   if(LEDMode){
  //     ledSchemes.get(schemeIndex).run();
  //     m_LED.setData(m_LEDBuffer);
  //   } else {
  //     System.out.println("LED Mode Somehow Not On, Cannot Change Schema!");
  //   }
  // }
  // public void loadNextScheme(){
  //   if(LEDMode){
  //     schemeIndex++;
  //     checkSchemeIndex();
  //     ledSchemes.get(schemeIndex).run();
  //     m_LED.setData(m_LEDBuffer);
  //   } else {
  //     System.out.println("LED Mode Not On, Cannot Change Schema!");
  //   }
  // }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void enableLedControl(){
    LEDMode = true;
  }

  public void disableLedControl(){
    LEDMode = false;
  }

  public AddressableLEDBuffer getBuffer(){
    return m_LEDBuffer;
  }

  
}
