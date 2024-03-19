// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Prototype extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkFlex m_ShooterMotor;
  private CANSparkFlex m_PrototypeMotor1;
  private CANSparkFlex m_PrototypeMotor2;
  private RelativeEncoder m_Encoder1;
  private RelativeEncoder m_Encoder2;

  private ShuffleboardTab teleopTab;
  public Prototype() {
    m_PrototypeMotor1 = new CANSparkFlex(51, MotorType.kBrushless);
    m_PrototypeMotor2 = new CANSparkFlex(52, MotorType.kBrushless);
  }
 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }


  public void setOutput(Supplier<Double> speedSupplier) {
    m_PrototypeMotor1.set(speedSupplier.get() * 0.2);
    m_PrototypeMotor2.set(-speedSupplier.get() * 0.2);
  }

  public void runCharacterizationVolts(double volts) {
    m_ShooterMotor.setVoltage(volts);
  }

  public CANSparkFlex[] getMotor() {
    return new CANSparkFlex[] {m_PrototypeMotor1, m_PrototypeMotor2};
  }

}
