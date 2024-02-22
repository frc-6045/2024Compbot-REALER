// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class AngleSubSystem extends SubsystemBase {
  private CANSparkFlex m_AngleMotor;
  private AbsoluteEncoder m_AngleEncoder;
  private SparkPIDController m_PidController;
  private double encoderOffset = 0.015;
  /** Creates a new AngleSubSystem. */
  public AngleSubSystem() {
    m_AngleMotor = new CANSparkFlex(ShooterConstants.kAngleControlCANID, MotorType.kBrushless);
    m_AngleMotor.restoreFactoryDefaults();
    m_AngleMotor.setInverted(true);
    m_AngleEncoder = m_AngleMotor.getAbsoluteEncoder(Type.kDutyCycle);
    m_PidController = m_AngleMotor.getPIDController();
    m_PidController.setFeedbackDevice(m_AngleEncoder);
    m_PidController.setP(3.5);
    m_PidController.setI(0.0);
    m_PidController.setD(50.0);
    m_PidController.setIZone(0.0);
    m_PidController.setFF(0.0);
    m_PidController.setOutputRange(-0.5, 0.5);
    m_AngleMotor.burnFlash();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Encoder", getAngleDeg());
  }

  public void runClosedLoop(double deg){
    MathUtil.clamp(deg, 0, 60);
    double rotations = (deg/360) + encoderOffset;
    System.out.println(rotations);
    m_PidController.setReference(rotations, CANSparkFlex.ControlType.kPosition);
  }
  
  public void runOpenLoop(double speed){
    m_AngleMotor.set(speed);
  }

  public double getAngleDeg() {
    return 360*m_AngleEncoder.getPosition();
  }

}
