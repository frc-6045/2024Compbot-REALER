// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

//This will probably be real scuffed, sorry
public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private final CANSparkFlex m_ClimbMotor;
  private final DigitalInput m_LimitSwitch1;
  private final DigitalInput m_LimitSwitch2;
  
  
  public Climber() {
    m_ClimbMotor = new CANSparkFlex(ClimbConstants.kClimbMotorCanId, MotorType.kBrushless);
    m_LimitSwitch1 = new DigitalInput(9);
    m_LimitSwitch2 = new DigitalInput(1);
    m_ClimbMotor.setSmartCurrentLimit(50);

    m_ClimbMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runMotors(Supplier<Double> speedSupplier){
    m_ClimbMotor.set(-speedSupplier.get());
  }

  public DigitalInput getLimitSwitch1(){
    return m_LimitSwitch1;
  }

  public  DigitalInput getLimitSwitch2(){
    return m_LimitSwitch2;
  }
}
