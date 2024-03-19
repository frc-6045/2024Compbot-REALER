// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Amp extends SubsystemBase {
  /** Creates a new Trap. */
  private final CANSparkFlex m_AmpMotor;
  public Amp() {
    m_AmpMotor = new CANSparkFlex(ClimbConstants.kAmpMotorCanId, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public CANSparkFlex getAmpMotor(){
    return m_AmpMotor;
  }

  public void runMotors(Supplier<Double> speedSupplier){
    m_AmpMotor.set(speedSupplier.get());
  }
}
