// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;

public class TransLeds extends Command {
  /** Creates a new TransLeds. */
  private final LEDs m_leds;
  public TransLeds(LEDs leds) {
    m_leds = leds;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      int i = 0;
      int colorIndex = 0;

      while(i < m_leds.getBuffer().getLength()){
        if(colorIndex > 4){
          colorIndex = 0;
        }
        System.out.println("colorIndex: " + colorIndex);
        if(colorIndex == 0){
          m_leds.setPixelColor(i, 91, 206, 250);
        } else if(colorIndex == 1){
          m_leds.setPixelColor(i, 245, 169, 184);
        } else if(colorIndex == 2){
          m_leds.setPixelColor(i, 255, 255, 255);
        } else if(colorIndex == 3){
          m_leds.setPixelColor(i, 245, 169, 184);
        } else {
          m_leds.setPixelColor(i, 91, 206, 250);
        }
        // switch(colorIndex) {
        //   case 0: 
        //     m_leds.setPixelColor(i, 91, 206, 250);
        //   case 1: 
        //       m_leds.setPixelColor(i, 245, 169, 184);
        //   case 2: 
        //       m_leds.setPixelColor(i, 255, 255, 255);
        //   case 3: 
        //       m_leds.setPixelColor(i, 245, 169, 184);
        //   case 4: 
        //       m_leds.setPixelColor(i, 91, 206, 250);
        // }
        m_leds.setPixelColor(1, 245, 169, 184);
        i++;
        colorIndex++;
      }
      m_leds.setData();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}