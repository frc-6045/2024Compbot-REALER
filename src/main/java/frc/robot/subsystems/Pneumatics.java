// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class Pneumatics extends SubsystemBase {
  /** Creates a new Pneumatics. */
  private final Compressor m_Compressor;
  private final Solenoid m_IntakeSolenoid;
  private final Solenoid m_BrakeSolenoid;
  private boolean compressorEnabled;
  public Pneumatics() {
    m_Compressor = new Compressor(PneumaticsConstants.kPneumaticsModuleCANID, PneumaticsModuleType.REVPH);
    m_IntakeSolenoid = new Solenoid(PneumaticsConstants.kPneumaticsModuleCANID, PneumaticsModuleType.REVPH, PneumaticsConstants.kIntakeSolenoidSingleChannel);
    m_BrakeSolenoid = new Solenoid(PneumaticsConstants.kPneumaticsModuleCANID, PneumaticsModuleType.REVPH, PneumaticsConstants.kBrakeSolenoidSingleChannel);
    m_Compressor.enableAnalog(0, 115);
    m_Compressor.disable();
    compressorEnabled = false;
    System.out.println("enabled compressor");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void enableCompressor(){
    m_Compressor.enableAnalog(100, 115);
  }
  public void disableCompressor(){
    m_Compressor.disable();
  }

  public void ActutateIntakeSolenoid(boolean isOn) {
    m_IntakeSolenoid.set(isOn);
  }

  public void ActutateBrakeSolenoid(boolean isOn) {
    m_BrakeSolenoid.set(isOn);
  }
  
  public void ToggleIntakeSolenoids(){
    m_IntakeSolenoid.toggle();
  }

  public void ToggleBrakeSolenoids(){
    m_BrakeSolenoid.toggle();
  }

  public Solenoid getIntakeSolenoid(){
    return m_IntakeSolenoid;
  }

  public Solenoid getBrakeSolenoid(){
    return m_BrakeSolenoid;
  }

  
}
