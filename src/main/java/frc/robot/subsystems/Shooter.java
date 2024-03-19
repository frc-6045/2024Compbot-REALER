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

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkFlex m_BottomShooterMotor;
  private CANSparkFlex m_TopShooterMotor;
  private RelativeEncoder m_Encoder;

  private ShuffleboardTab teleopTab;
  public Shooter() {
    m_BottomShooterMotor = new CANSparkFlex(ShooterConstants.kBottomShooterMotorCANID, MotorType.kBrushless);
    m_TopShooterMotor = new CANSparkFlex(ShooterConstants.kTopShooterMotorCANID, MotorType.kBrushless);

    m_Encoder = m_BottomShooterMotor.getEncoder(); //TODO: we might want to swap out for the throughbore encoder here
    m_BottomShooterMotor.setInverted(true);
    m_BottomShooterMotor.burnFlash();
  }
 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }



  public void runMotors(Supplier<Double> speedSupplier) {
    m_BottomShooterMotor.set(speedSupplier.get());
    m_TopShooterMotor.set(speedSupplier.get());
  }

  public double getCharacterizationVelocity(){
    return m_Encoder.getVelocity() * 2 * Math.PI; //rads per sec (also this might break stuff by being rads per sec! guess we will see)
  }

  public void runCharacterizationVolts(double volts) {
    m_BottomShooterMotor.setVoltage(volts);
    m_TopShooterMotor.setVoltage(volts);
  }

  public CANSparkFlex[] getMotors() {
    CANSparkFlex[] array = {m_BottomShooterMotor, m_TopShooterMotor};
    return array;
  }

}
