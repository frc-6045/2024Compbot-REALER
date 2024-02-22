// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.openloop;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AngleSubSystem;

public class AngleOpenLoop extends Command {
  /** Creates a new AngleOpenLoop. */
  private final AngleSubSystem m_SubSystem;
  private DoubleSupplier m_speed;
  public AngleOpenLoop(DoubleSupplier speed, AngleSubSystem subsystem) {
    m_speed = speed;
    m_SubSystem = subsystem;
    addRequirements(m_SubSystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_SubSystem.runOpenLoop(m_speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_SubSystem.runOpenLoop(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
