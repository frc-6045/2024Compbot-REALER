// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closedloop;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.SparkRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class PIDShooterNoIntake extends Command {
  /** Creates a new PIDShooter. */
  private final Shooter m_Shooter;
  private final Feeder m_Feeder;
  private SparkPIDController m_BottomPIDController;
  private SparkPIDController m_TopPIDController;

  private SimpleMotorFeedforward m_Feedforward;
  private final CANSparkFlex bottomShooterMotor;
  private final CANSparkFlex topShooterMotor; 
  private final RelativeEncoder bottomEncoder;
  private final RelativeEncoder topEncoder;
  private double setpoint;
  private boolean atSetpoint;
  private boolean timerSet;
  private Timer timer;
  public PIDShooterNoIntake(Shooter shooter, Feeder feeder, double setpoint, boolean isSlow) {
    m_Shooter = shooter;
    m_Feeder = feeder;
    bottomShooterMotor = shooter.getMotors()[0];
    topShooterMotor = shooter.getMotors()[1];
    bottomEncoder = bottomShooterMotor.getEncoder(); //TODO check type of encoder
    topEncoder = topShooterMotor.getEncoder();
    this.setpoint = setpoint;
    m_TopPIDController = topShooterMotor.getPIDController();
    m_BottomPIDController = bottomShooterMotor.getPIDController();
    if(!isSlow){
    m_BottomPIDController.setP(ShooterConstants.kShooterP);
    m_BottomPIDController.setI(ShooterConstants.kShooterI);
    m_BottomPIDController.setD(ShooterConstants.kShooterD);
    m_TopPIDController.setP(ShooterConstants.kShooterP);
    m_TopPIDController.setI(ShooterConstants.kShooterI);
    m_TopPIDController.setD(ShooterConstants.kShooterD);
    } else {
    m_BottomPIDController.setP(ShooterConstants.kShooterAmpP);
    m_BottomPIDController.setI(ShooterConstants.kShooterAmpI);
    m_BottomPIDController.setD(ShooterConstants.kShooterAmpD);
    m_TopPIDController.setP(ShooterConstants.kShooterAmpP);
    m_TopPIDController.setI(ShooterConstants.kShooterAmpI);
    m_TopPIDController.setD(ShooterConstants.kShooterAmpD);
    }
    m_BottomPIDController.setIZone(0);
    m_BottomPIDController.setFF(.0001);
    m_BottomPIDController.setOutputRange(-1, 1);
    m_TopPIDController.setIZone(0);
    m_TopPIDController.setFF(.0001);
    m_TopPIDController.setOutputRange(-1, 1);

    m_BottomPIDController.setFeedbackDevice(bottomEncoder);
    m_TopPIDController.setFeedbackDevice(topEncoder);
    m_Feedforward = new SimpleMotorFeedforward(.17674,.00030); //TODO characterization
    atSetpoint = false;
    timer = new Timer();
    timerSet = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Shooter, m_Feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    atSetpoint = false;
    timerSet = false;
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

   
   
    m_BottomPIDController.setReference(setpoint, ControlType.kVelocity, 0, m_Feedforward.calculate(setpoint));
    m_TopPIDController.setReference(setpoint, ControlType.kVelocity, 0, m_Feedforward.calculate(setpoint)); //TODO: characterization for feedforward

    
   System.out.println(bottomEncoder.getVelocity());
    if(bottomEncoder.getVelocity() <= ShooterConstants.kShooterLaunchRPM){
      if(!timerSet){

        timer.reset();
        timer.start();
        timerSet = true;
      }
      m_Feeder.runMotors(() -> {return FeederConstants.kFeederSpeed;});              
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //shooterMotor.set(0);
    bottomShooterMotor.set(0);
    topShooterMotor.set(0);
    m_Feeder.runMotors(() -> {return 0.0;});
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(timer.get());
    if(timer.get() > 0.9){ //lmao
      return true;
    }
    return false;
  }

  public boolean getAtSetpoint() {
    return atSetpoint;
  }
}
