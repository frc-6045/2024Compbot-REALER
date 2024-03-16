// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closedloop;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.util.PoseMath;
public class AutoAlign extends Command {
  /** Creates a new AutoAlign. */
  private DriveSubsystem m_drive;
  private Supplier<Pose2d> poseSupplier;
  private Pose2d goalPose;
  private Rotation2d robotToGoalAngle;
  private Rotation2d targetVehicleDirection;
  private ProfiledPIDController headingController;

  public AutoAlign(DriveSubsystem drive, Supplier<Pose2d> poseSupplier) {
    m_drive = drive;
    this.poseSupplier = poseSupplier;
    headingController = new ProfiledPIDController(0, 0, 0, null); //TODO: figure out these values
    headingController.enableContinuousInput(-180, 180);
    headingController.setTolerance(2);
    targetVehicleDirection = new Rotation2d();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    headingController.reset(m_drive.getPose().getRotation().getDegrees());
    Pose2d aimPose = new Pose2d(m_drive.getPose().getTranslation(), new Rotation2d());
     Translation2d predictedVehicleFixedToTargetTranslation = // i have no idea what this means
        PoseMath.inverse(aimPose).transformBy(PoseMath.toTransform2d(FieldConstants.centerSpeakerOpening.toTranslation2d())).getTranslation(); //TODO: may want to undo all of this weird flipping

        targetVehicleDirection = predictedVehicleFixedToTargetTranslation.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = headingController.calculate(m_drive.getPose().getRotation().getDegrees(),
      targetVehicleDirection.getDegrees());
    
    m_drive.drive(0, 0, speed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, true);
    System.out.println("auto align ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return headingController.atGoal();
  }
}

// public static ChassisSpeeds autoAlign(Supplier<Pose2d> poseSupplier, SwerveDrivePoseEstimator swerveOdometry,boolean slowMode){
//   ChassisSpeeds commandSpeeds = new ChassisSpeeds();
//   Pose2d goalPose = poseSupplier.get();
//   Pose2d currentPose = swerveOdometry.getEstimatedPosition();
//   Rotation2d robotToGoalAngle = goalPose.getTranslation().minus(currentPose.getTranslation()).getAngle();

//   ProfiledPIDController anglePIDController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0)); //TODO: add PID values
//   if(!slowMode) {
//       anglePIDController.setConstraints(new TrapezoidProfile.Constraints(0, 0));
//   } else {
//       anglePIDController.setConstraints(new TrapezoidProfile.Constraints(0, 0));
//   }
//   anglePIDController.enableContinuousInput(-180, 180);

//   double angleSpeed = anglePIDController.calculate(currentPose.getRotation())
//   return commandSpeeds;
// }
