// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closedloop;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.util.PoseMath;
import frc.robot.util.Vision;

public class AutoAlignWithDrive extends Command {
  /** Creates a new AutoAlignWithDrive. */
  private final DriveSubsystem m_DriveSubsystem;
  private final Supplier<Double> m_XSpeedSupplier;
  private final Supplier<Double> m_YSpeedSupplier;
  private final PIDController m_ThetaController;
  public AutoAlignWithDrive(DriveSubsystem driveSubsystem, Supplier<Double> xSpeed, Supplier<Double> ySpeed) {
    m_DriveSubsystem = driveSubsystem;
    m_XSpeedSupplier = xSpeed;
    m_YSpeedSupplier = ySpeed;
    addRequirements(m_DriveSubsystem);

    m_ThetaController = new PIDController(2, 0, 0.001);
    m_ThetaController.enableContinuousInput(-Math.PI, Math.PI);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ThetaController.setSetpoint(PoseMath.getTargetAngleRobotToTarget(m_DriveSubsystem.getPose(), m_DriveSubsystem.getChassisSpeeds()));
    double thetaOutput = m_ThetaController.calculate(m_DriveSubsystem.getPose().getRotation().rotateBy(Rotation2d.fromRadians(Math.PI)).getRadians());
    m_DriveSubsystem.drive(m_XSpeedSupplier.get(), m_YSpeedSupplier.get(), thetaOutput, true); //might have to disable rateLimiting
    Vision.setDisplacementToTargetAngle(m_ThetaController.getPositionError());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.drive(0,0,0,true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
