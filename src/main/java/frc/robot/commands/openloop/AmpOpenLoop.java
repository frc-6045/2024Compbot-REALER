// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.openloop;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Amp;

public class AmpOpenLoop extends Command {
  /** Creates a new AmpOpenLoop. */
  private final Amp m_amp;
  private Supplier<Double> speedSupplier;
  public AmpOpenLoop(Amp amp, Supplier<Double> speedSupplier) {
    m_amp = amp;
    this.speedSupplier = speedSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_amp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_amp.runMotors(speedSupplier);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_amp.runMotors(() -> {return 0.0;});
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
