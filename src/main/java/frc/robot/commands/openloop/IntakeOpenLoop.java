// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.openloop;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;

public class IntakeOpenLoop extends Command {
  private Intake m_Intake;
  private LEDs m_LEDs;
  private Supplier<Double> speedSupplier;
  /** Creates a new IntakeOpenLoop. */
  public IntakeOpenLoop(Intake intake, LEDs leds, Supplier<Double> speedSupplier) {
    m_Intake = intake;
    m_LEDs = leds;
    this.speedSupplier = speedSupplier;
    addRequirements(m_Intake, m_LEDs);
     // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Intake.checkNote();
    if(m_Intake.hasNote()){
      m_LEDs.setColor(23, 252, 3);
      // m_Intake.stopIntake();
    } else {
      m_LEDs.setColor(39, 2, 201);
      // m_Intake.runIntake(speedSupplier);
    }
    m_Intake.runIntake(speedSupplier);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
