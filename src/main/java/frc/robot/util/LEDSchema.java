// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import frc.robot.subsystems.LEDs;

/** Add your docs here. */
public class LEDSchema {
    private LEDs leds;
    public LEDSchema(LEDs leds){
        this.leds = leds;
    }
    public void TransSchema(){
        int i = 0;
        int colorIndex = 0;
        while(i < leds.getBuffer().getLength()){
          if(colorIndex > 4){
            colorIndex = 0;
          }
          switch(colorIndex) {
            case 0: 
              leds.setPixelColor(i, 91, 206, 250);
            case 1: 
                leds.setPixelColor(i, 245, 169, 184);
            case 2: 
                leds.setPixelColor(i, 255, 255, 255);
            case 3: 
                leds.setPixelColor(i, 245, 169, 184);
            case 4: 
                leds.setPixelColor(i, 91, 206, 250);
            default: 
                leds.setPixelColor(i, 255, 255, 255);
          }
          i++;
          colorIndex++;
        }
      }
  
      public void GaySchema(){
        int i = 0;
        int colorIndex = 0;
        while(i < leds.getBuffer().getLength()){
          if(colorIndex > 5){
            colorIndex = 0;
          }
        
        switch(colorIndex) {
          case 0: 
            leds.setPixelColor(i, 228, 3, 3);
          case 1: 
            leds.setPixelColor(i, 255, 140, 0);
          case 2: 
            leds.setPixelColor(i, 255, 237, 0);
          case 3: 
            leds.setPixelColor(i, 0, 128, 38);
          case 4: 
            leds.setPixelColor(i, 	36, 64, 142);
          case 5:
            leds.setPixelColor(i, 115, 41, 130);
          default: 
            leds.setPixelColor(i, 255, 255, 255);
        }
        i++;
        colorIndex++;
      }
    }
}
