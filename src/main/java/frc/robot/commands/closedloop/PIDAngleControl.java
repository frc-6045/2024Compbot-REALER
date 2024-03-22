// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closedloop;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.AngleController;
import frc.robot.subsystems.Shooter;


public class PIDAngleControl extends Command {
  /** Creates a new PIDAngleControl. */

  private final AngleController m_AngleController;
  private final PIDController m_AnglePIDController;
  private Supplier<Double> setpoint;
  private double actualSetpoint;
  public PIDAngleControl(AngleController angleController, Supplier<Double> setpoint) {
    m_AngleController = angleController;
    this.setpoint = setpoint;
    System.out.println("first setpoint : " + setpoint.get());
    m_AnglePIDController = new PIDController(ShooterConstants.kShooterAngleP, ShooterConstants.kShooterAngleI, ShooterConstants.kShooterAngleD);
    m_AnglePIDController.setTolerance(.003); //was .01//who care
    m_AnglePIDController.enableContinuousInput(0,1);
    //m_AnglePIDController.disableContinuousInput();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_AngleController);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("PID angel controll scheduled 8)");
    System.out.println("setpoint : " + setpoint.get());
    actualSetpoint = setpoint.get();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("error: " + m_AnglePIDController.getPositionError());
    System.out.println("setpoint: " + setpoint.get());
    double feedforward = ShooterConstants.kAngleFeedforward; // just to have TODO: maybe do characterization??
    double speed;
    // if(m_AngleController.getAngleEncoder().getPosition() > .75){
    //   speed = -.10;
    // } else {
      speed = m_AnglePIDController.calculate(m_AngleController.getAngleEncoder().getPosition(), actualSetpoint);
      if(speed > 0){
        speed = speed + feedforward;
      } else {
        speed = speed - feedforward;
      }
    m_AngleController.getAngleMotor().set(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("pidanglecontrol is finished");
    m_AngleController.getAngleMotor().set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return true;
    return m_AnglePIDController.atSetpoint();

   
  }
}
