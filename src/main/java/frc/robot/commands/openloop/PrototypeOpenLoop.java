// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.openloop;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Prototype;

public class PrototypeOpenLoop extends Command {
  /** Creates a new PrototypeOpenLoop. */
  private final Prototype m_Prototype;
  private final Supplier<Double> m_SpeedSupplier;
  public PrototypeOpenLoop(Prototype m_Prototype, Supplier<Double> m_SpeedSupplier) {

    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Prototype = m_Prototype;
    this.m_SpeedSupplier = m_SpeedSupplier;
    addRequirements(m_Prototype);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Prototype.setOutput(m_SpeedSupplier);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Prototype.setOutput(()-> 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
