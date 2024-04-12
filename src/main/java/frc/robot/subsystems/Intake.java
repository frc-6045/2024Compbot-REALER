// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;




import java.util.function.Supplier;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final CANSparkFlex m_IntakeMotor;
  private final CANSparkFlex m_IndexerMotor;
  private final TimeOfFlight m_TimeOfFlightSensor;
  private boolean hasNote;
  public Intake() {
    m_IntakeMotor = new CANSparkFlex(IntakeConstants.kIntakeCANID, MotorType.kBrushless);
    //m_Pneumatics.ActutateIntakeSolenoid();
    m_IndexerMotor = new CANSparkFlex(IntakeConstants.kIndexerCANID, MotorType.kBrushless);
    m_IntakeMotor.restoreFactoryDefaults();
    m_IndexerMotor.restoreFactoryDefaults();
    m_IntakeMotor.setSmartCurrentLimit(IntakeConstants.kIntakeCurrentLimit);
    m_IndexerMotor.setSmartCurrentLimit(IntakeConstants.kIndexerCurrentLimit);
    m_IntakeMotor.setIdleMode(IdleMode.kBrake);
    m_IndexerMotor.setIdleMode(IdleMode.kBrake);

    m_IntakeMotor.burnFlash();
    m_IndexerMotor.burnFlash();

    m_TimeOfFlightSensor = new TimeOfFlight(IntakeConstants.kTimeOfFlightCANID);
    m_TimeOfFlightSensor.setRangingMode(RangingMode.Short, 24);
  }

  public void runIntake(Supplier<Double> speed){
    if(speed.get() <= IntakeConstants.kIntakeSpeed){
      m_IntakeMotor.set(-speed.get());
      m_IndexerMotor.set(speed.get());
    } else {
      m_IntakeMotor.set(-IntakeConstants.kIntakeSpeed);
      m_IndexerMotor.set(IntakeConstants.kIntakeSpeed);
    }
  }

   public void runIntakeNoIndexer(Supplier<Double> speed){
    if(speed.get() <= IntakeConstants.kIntakeSpeed){
      m_IntakeMotor.set(-speed.get());
    } else {
      m_IntakeMotor.set(-IntakeConstants.kIntakeSpeed);
    }
  }

  public void testRunMotors(Supplier<Double> speed1, Supplier<Double> speed2){
    if(speed1.get() <= IntakeConstants.kIntakeSpeed){
      m_IntakeMotor.set(speed1.get());
    } else {
      m_IntakeMotor.set(IntakeConstants.kIntakeSpeed);
    }
    if(speed2.get() <= IntakeConstants.kIndexerSpeed){
      m_IndexerMotor.set(speed2.get());
    } else {
      m_IndexerMotor.set(IntakeConstants.kIndexerSpeed);
    }
  }

  public void intakeIn(){
    System.out.println("intake in");
    m_IntakeMotor.set(-IntakeConstants.kIntakeSpeed);
    m_IndexerMotor.set(IntakeConstants.kIntakeSpeed);

  }

  public void intakeInNoFeeder(){
    m_IntakeMotor.set(-IntakeConstants.kIntakeSpeed);
  }
  public void intakeOut(){
    m_IntakeMotor.set(IntakeConstants.kIntakeSpeed);
    m_IndexerMotor.set(-IntakeConstants.kIntakeSpeed);
  }

  public void stopIntake() {
    m_IntakeMotor.set(0);
    m_IndexerMotor.set(0);
  }

   
  public void checkNote(){
    System.out.println("sensor range: " + m_TimeOfFlightSensor.getRange());
    System.out.println("sensor std dev: " + m_TimeOfFlightSensor.getRangeSigma());
    System.out.println("status: " + m_TimeOfFlightSensor.getStatus().toString());
    if(m_TimeOfFlightSensor.getRange() < 350){
      hasNote = true;
    } else {
      hasNote = false;
    }
  }

  public boolean hasNote() {
    return hasNote;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
