// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closedloop;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AngleSubSystem;

public class AngleSetpoint extends Command {
  /** Creates a new AngleSetpoint. */
  private final AngleSubSystem m_Subsystem;
  private double m_setpoint;
  public AngleSetpoint(double setpoint, AngleSubSystem subsystem) {
    m_Subsystem = subsystem;
    m_setpoint = setpoint;
    addRequirements(m_Subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriverStation.reportError("init _AngleSetPoint: " + m_setpoint, false);
    m_Subsystem.runClosedLoop(m_setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
