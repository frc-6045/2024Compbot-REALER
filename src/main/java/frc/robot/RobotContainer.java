// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.closedloop.HoldAngle;
import frc.robot.commands.closedloop.PIDAngleControl;
import frc.robot.commands.closedloop.PIDShooter;
import frc.robot.commands.closedloop.PIDShooterNoIntake;
import frc.robot.commands.openloop.AmpOpenLoop;
import frc.robot.commands.openloop.FeederOpenLoop;
import frc.robot.commands.openloop.IntakeOpenLoop;
import frc.robot.commands.openloop.ShooterOpenLoop;
import frc.robot.subsystems.AngleController;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Prototype;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Amp;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.util.LookupTables;
import frc.robot.util.PoseMath;

public class RobotContainer {
private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
private final XboxController m_driverController = new XboxController(0);
private final XboxController m_operatorController = new XboxController(1);
private final Shooter m_Shooter = new Shooter();
private final Feeder m_Feeder = new Feeder();
private final Pneumatics m_Pneumatics = new Pneumatics();
private final Intake m_Intake = new Intake();
private final Climber m_Climber = new Climber();
private final AngleController m_AngleController = new AngleController();
private final Amp m_Amp = new Amp();
private final Prototype m_Prototype = new Prototype();
private final LEDs m_LEDs = new LEDs();

private Autos m_Autos;
private ShuffleboardTab teleopTab = Shuffleboard.getTab("teleOp");
public RobotContainer() {
  NamedCommands.registerCommand("SpeakerSetpoint", new ParallelDeadlineGroup(new WaitCommand(1.00), new PIDAngleControl(m_AngleController,() -> {return ShooterConstants.kSubwooferAngleSetpoint;})));
  NamedCommands.registerCommand("AmpSetpoint", new ParallelDeadlineGroup(new WaitCommand(2.00), new PIDAngleControl(m_AngleController, () -> {return ShooterConstants.kAngleAmpHandoffSetpoint;})));
  NamedCommands.registerCommand("FarStealShootSetpoint1", new ParallelDeadlineGroup(new WaitCommand(1.00), new PIDAngleControl(m_AngleController, () -> {return 0.330;}))); // 0.335
  NamedCommands.registerCommand("FarStealShootSetpoint2", new ParallelDeadlineGroup(new WaitCommand(1.00), new PIDAngleControl(m_AngleController, () -> {return 0.349;}))); // 0.348
  NamedCommands.registerCommand("FarStealShootSetpoint3", new ParallelDeadlineGroup(new WaitCommand(1.00), new PIDAngleControl(m_AngleController, () -> {return 0.3621;}))); // 0.3591
  NamedCommands.registerCommand("RingPassthrough", new ParallelDeadlineGroup(new WaitCommand(1.40), new ShooterOpenLoop(m_Shooter, () -> {return -ShooterConstants.kAmpShooterMaxSpeed;}), new FeederOpenLoop(m_Feeder, () -> {return -FeederConstants.kAmpFeederAutoSpeed;}), new AmpOpenLoop(m_Amp, () -> -ClimbConstants.kAmpHandoffMaxAutoSpeed), new IntakeOpenLoop(m_Intake, m_LEDs, () -> -IntakeConstants.kIntakeAutoSpeed)));
  NamedCommands.registerCommand("AutoAngle", new ParallelDeadlineGroup(new WaitCommand(0.5), new PIDAngleControl(m_AngleController,() -> {return LookupTables.getAngleValueAtDistance(PoseMath.getDistanceToSpeakerBack(m_driveSubsystem.getPose()));})));
  NamedCommands.registerCommand("PIDShooting", new ParallelDeadlineGroup(new WaitCommand(1.4), new PIDShooter(m_Shooter, m_Feeder, m_Intake, -6000, false)));
  NamedCommands.registerCommand("RunAmp", new ParallelDeadlineGroup(new WaitCommand(0.5), new AmpOpenLoop(m_Amp, () -> {return -ClimbConstants.kAmpHandoffMaxSpeed;})));
  NamedCommands.registerCommand("IntakeAndShoot", new ParallelDeadlineGroup(new WaitCommand(2.0), new IntakeOpenLoop(m_Intake, m_LEDs, () -> -IntakeConstants.kIntakeAutoSpeed), new PIDShooterNoIntake(m_Shooter, m_Feeder, -6000, false)));

  //Commented these out because they're old
  NamedCommands.registerCommand("AngleAndShoot", new SequentialCommandGroup(new PIDAngleControl(m_AngleController, () -> {return LookupTables.getAngleValueAtDistance(PoseMath.getDistanceToSpeakerBack(m_driveSubsystem.getPose()));}), new PIDShooter(m_Shooter, m_Feeder, m_Intake, -6000, false)));
  NamedCommands.registerCommand("AngleAndShootClose", new SequentialCommandGroup(new PIDAngleControl(m_AngleController, () -> {return 0.797;}), new PIDShooter(m_Shooter, m_Feeder, m_Intake, -3000, true)));
  NamedCommands.registerCommand("ShootClose", new PIDShooter(m_Shooter, m_Feeder, m_Intake, -3000, true));
  NamedCommands.registerCommand("ShootFromDistance", new SequentialCommandGroup(new PIDAngleControl(m_AngleController, () -> {return LookupTables.getAngleValueAtDistance(PoseMath.getDistanceToSpeakerBack(m_driveSubsystem.getPose()));}), new PIDShooter(m_Shooter, m_Feeder, m_Intake, -3000, true)));
  NamedCommands.registerCommand("StartingAngleAndShootClose", new SequentialCommandGroup(new PIDAngleControl(m_AngleController, () -> {return ShooterConstants.kSubwooferAngleSetpoint;}), new PIDShooter(m_Shooter, m_Feeder, m_Intake, -3000, true)));
  NamedCommands.registerCommand("AngleAndShoot4Ring", new SequentialCommandGroup(new PIDAngleControl(m_AngleController, () -> {return ShooterConstants.kAngle4RingSetpoint;}), new PIDShooter(m_Shooter, m_Feeder, m_Intake, -3000, true)));

  NamedCommands.registerCommand("DisableCompressor", new InstantCommand(() -> m_Pneumatics.disableCompressor()));
  NamedCommands.registerCommand("IntakeOut", new SequentialCommandGroup(new ParallelDeadlineGroup(new WaitCommand(1.5), new RunCommand(() -> {m_Intake.intakeIn();}, m_Intake)), new InstantCommand(() -> {m_Intake.stopIntake();}, m_Intake)));
  NamedCommands.registerCommand("IntakeIn", new SequentialCommandGroup(new ParallelDeadlineGroup(new WaitCommand(1.5), new RunCommand(() -> {m_Intake.intakeOut();}, m_Intake)), new InstantCommand(() -> {m_Intake.stopIntake();}, m_Intake)));
  NamedCommands.registerCommand("ActuateIntakeDown", new ParallelDeadlineGroup(new WaitCommand(0.1), new RunCommand(() -> {m_Pneumatics.ActutateIntakeSolenoid(true);})));
  NamedCommands.registerCommand("ActuateIntakeUp", new ParallelDeadlineGroup(new WaitCommand(0.1), new RunCommand(() -> {m_Pneumatics.ActutateIntakeSolenoid(false);})));
  m_Autos = new Autos(m_driveSubsystem, m_Feeder, m_Intake, m_Pneumatics, m_Shooter);  
  
    m_driveSubsystem.setDefaultCommand(
    new RunCommand(
          () -> m_driveSubsystem.drive( 
              MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.15), 
              MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.15),
              MathUtil.applyDeadband(-m_driverController.getRightX(), 0.20),
              true),
          m_driveSubsystem)
    );
    
    m_AngleController.setDefaultCommand(
      new HoldAngle(m_AngleController, () -> {return m_AngleController.getAngleEncoder().getPosition();}));

    configureBindings();
    teleopTab.addDouble("Right Trigger Axis", m_driverController::getRightTriggerAxis);
    teleopTab.addDouble("Left Trigger Axis", m_driverController::getLeftTriggerAxis);
    teleopTab.addDouble("Shooter RPM", () -> {return m_Shooter.getMotors()[0].getEncoder().getVelocity();});
    teleopTab.addDouble("hood position", () -> {return m_AngleController.getAngleEncoder().getPosition();});
  }

  private void configureBindings() {
    Bindings.InitBindings(m_driverController, m_operatorController, 
    m_driveSubsystem, m_Shooter, 
    m_Feeder, m_Pneumatics, 
    m_AngleController, m_Intake, 
    m_Climber, m_Amp,
    m_Prototype, m_LEDs);
  }

  public Command getAutonomousCommand() {
    return m_Autos.getAutonomousCommand();
  }
}