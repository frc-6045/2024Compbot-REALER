// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.closedloop.AimAtSpeaker;
import frc.robot.commands.closedloop.AutoAlign;
import frc.robot.commands.closedloop.AutoAlignWithDrive;
import frc.robot.commands.closedloop.HoldAngle;

import frc.robot.commands.closedloop.PIDAngleControl;
import frc.robot.commands.closedloop.PIDShooter;
import frc.robot.commands.closedloop.PIDShooterNoIndexer;
import frc.robot.commands.leds.GayLeds;
import frc.robot.commands.leds.TransLeds;
import frc.robot.commands.openloop.AngleOpenLoop;
import frc.robot.commands.openloop.ClimberOpenLoop;
import frc.robot.commands.openloop.FeederOpenLoop;
import frc.robot.commands.openloop.IntakeOpenLoop;
import frc.robot.commands.openloop.PrototypeOpenLoop;
import frc.robot.commands.openloop.ShooterAndFeederOpenLoop;
import frc.robot.commands.openloop.ShooterOpenLoop;
import frc.robot.commands.openloop.AmpOpenLoop;
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

/** Add your docs here. */
// Henry's Comment
// Dominic
public class Bindings {

    private static boolean bCompressorEnabled = false;
    private static boolean bIntakeToggle = true;
    private static boolean bBrakeToggle = true;
    private static boolean bLedToggle = true;


    public Bindings() {}

        
        public static void InitBindings(XboxController driverController, XboxController operatorController,  XboxController ledController,
            DriveSubsystem driveSubsystem, 
            Shooter shooter,
            Feeder feeder,
            Pneumatics pneumatics, 
            AngleController angleController,
            Intake intake,
            Climber climber, 
            Amp amp,
            Prototype m_Prototype,
            LEDs leds){
        
        //new JoystickButton(driverController, XboxController.Button.kStart.value).onTrue(new InstantCommand(() -> { driveSubsystem.zeroHeading();}, driveSubsystem));
        new Trigger(() -> {return driverController.getStartButtonPressed();}).onTrue(new InstantCommand(() -> {driveSubsystem.zeroHeading();}, driveSubsystem));
        
        if(FieldConstants.kVisionEnable){
        if(FieldConstants.kAutoAlignStratToggle){
            //new Trigger(() -> {return driverController.getYButtonPressed();}).onTrue(new AimAtSpeaker(driveSubsystem));
            new Trigger(() -> {return driverController.getLeftBumper();}).whileTrue(new AutoAlignWithDrive(driveSubsystem, () -> {return MathUtil.applyDeadband(driverController.getLeftY(), 0.15);}, 
           () -> {return MathUtil.applyDeadband(-driverController.getLeftX(), 0.15);}));
        } else {
            new Trigger(() -> {return driverController.getYButtonPressed();}).onTrue(new AutoAlign(driveSubsystem, () -> {return new Pose2d(FieldConstants.centerSpeakerOpening.toTranslation2d(), new Rotation2d());}));
        }
    }

        //shooter
        //new Trigger(() -> {return operatorController.getAButton();}).whileTrue(new FeederOpenLoop(feeder, () -> {return FeederConstants.kFeederSpeed;}));
        
        //new Trigger(() -> {return driverController.getLeftTriggerAxis() > 0;}).whileTrue(new ShooterOpenLoop(shooter, operatorController::getLeftTriggerAxis));

        //new Trigger(() -> {return operatorController.getRightTriggerAxis() > 0;}).whileTrue(new ShooterAndFeederOpenLoop(shooter, feeder, operatorController::getRightTriggerAxis, operatorController::getRightTriggerAxis));
        
        new Trigger(() -> {return operatorController.getXButton();}).whileTrue(new PIDShooter(shooter, feeder, intake, -6000, ShooterConstants.kShootingLaunchRPM, false));

        new Trigger(() -> {return driverController.getLeftTriggerAxis() > 0;}).whileTrue(new PIDShooter(shooter, feeder, intake, -4000, ShooterConstants.kFeedingLaunchRPM, false));
        
        new Trigger(() -> {return operatorController.getAButton();}).onTrue(new PIDAngleControl(angleController, leds, () -> {return ShooterConstants.kAngleAmpHandoffSetpoint;}));

        new Trigger(() -> {return operatorController.getStartButtonPressed();}).onTrue(new PIDAngleControl(angleController, leds, () -> {return ShooterConstants.kSubwooferAngleSetpoint;}));
            
        if(FieldConstants.kVisionEnable){
        //could possibly still work with odometry instead of vision
         new Trigger(() -> {return operatorController.getBackButtonPressed();}).onTrue(new PIDAngleControl(angleController, leds, () -> {return LookupTables.getAngleValueAtDistance(PoseMath.getDistanceToSpeakerBack(driveSubsystem.getPose()));})); //3.9624 works
        // } else {
        // new Trigger(() -> {return operatorController.getBackButtonPressed();}).onTrue(new PIDAngleControl(angleController,() -> {return ShooterConstants.kAngleCloseSetpoint;})); //number returned is angle setpoint
         }  

        // new Trigger(() -> {return operatorController.getStartButtonPressed();}).onTrue(new PIDAngleControl(angleController,() -> {return ShooterConstants.kAnglePodiumSetpoint;})); //was kAngleMidSetpoint

       // new Trigger(() -> {return operatorController.getBButtonPressed();}).onTrue(new PIDAngleControl(angleController,() -> {return ShooterConstants.kAngleRestSetpoint;}));

       // new Trigger(() -> {return operatorController.getYButtonPressed();}).onTrue(new PIDAngleControl(angleController, () -> {return ShooterConstants.kAngleClimbSetpoint;}));

        //new Trigger(() -> {return driverController.getBackButtonPressed();}).onTrue(new TurnAndAim(angleController, driveSubsystem)); //3.9624 works

        
        //Angle Controller
        new Trigger(() -> {return operatorController.getYButton();}).whileTrue(new AngleOpenLoop(angleController, -ShooterConstants.kAngleControlMaxSpeed)).onFalse(new HoldAngle(angleController, () -> {return angleController.getAngleEncoder().getPosition();}));

        new Trigger(() -> {return operatorController.getBButton();}).whileTrue(new AngleOpenLoop(angleController, ShooterConstants.kAngleControlMaxSpeed)).onFalse(new HoldAngle(angleController, () -> {return angleController.getAngleEncoder().getPosition();}));

        //new Trigger(() -> {return operatorController.getYButton();}).whileTrue(new AngleOpenLoop(angleController, -ShooterConstants.kAngleControlMaxSpeed));
        
        //new Trigger(() -> {return operatorController.getBButton();}).whileTrue(new AngleOpenLoop(angleController, ShooterConstants.kAngleControlMaxSpeed));
        //Compressor Toggle
        new Trigger(() -> {return driverController.getRightBumper();}).onTrue(new InstantCommand(() -> {
        if(bCompressorEnabled){
            pneumatics.disableCompressor();
            bCompressorEnabled = false;
        } else {
            pneumatics.enableCompressor();
            bCompressorEnabled = true;
        }
        }, pneumatics));

        
        new Trigger(() -> {return operatorController.getPOV() == 90;}).onTrue(new InstantCommand(() -> {
            pneumatics.ActutateIntakeSolenoid(bIntakeToggle);
            bIntakeToggle = !bIntakeToggle;
        }));

        new Trigger(() -> {return driverController.getXButton();}).onTrue(new InstantCommand(() -> {
            pneumatics.ActutateBrakeSolenoid(bBrakeToggle);
            if(bBrakeToggle == true){
               leds.setColor(255, 0, 0);
            } else {
                leds.setColor(39, 2, 201);
            }
            bBrakeToggle = !bBrakeToggle;
        }));

    

         //new Trigger(() -> {return operatorController.getRightTriggerAxis() > .05;}).whileTrue(new IntakeOpenLoop(intake, operatorController::getRightTriggerAxis));
         //new Trigger(()-> driverController.getRightTriggerAxis() != 0).whileTrue(new PrototypeOpenLoop(m_Prototype, ()-> driverController.getRightTriggerAxis()).alongWith(new PrintCommand("stuff")));


         new Trigger(() -> {return operatorController.getLeftTriggerAxis() > .05;}).whileTrue(new IntakeOpenLoop(intake, leds, () -> {return -operatorController.getLeftTriggerAxis();}));
         new Trigger(() -> {return operatorController.getRightTriggerAxis() > .05;}).whileTrue(new IntakeOpenLoop(intake, leds, () -> {return operatorController.getRightTriggerAxis();}));

         new Trigger(() -> {return operatorController.getRightBumper();}).whileTrue(new ParallelCommandGroup(new ShooterOpenLoop(shooter, () -> {return ShooterConstants.kAmpShooterMaxSpeed;}), new FeederOpenLoop(feeder, () -> {return FeederConstants.kAmpFeederSpeed;}), new AmpOpenLoop(amp, () -> ClimbConstants.kAmpHandoffMaxSpeed), new IntakeOpenLoop(intake, leds, () -> IntakeConstants.kIntakeSlowSpeed)));
         new Trigger(() -> {return operatorController.getLeftBumper();}).whileTrue(new ParallelCommandGroup(new ShooterOpenLoop(shooter, () -> {return -ShooterConstants.kAmpShooterMaxSpeed;}), new FeederOpenLoop(feeder, () -> {return -FeederConstants.kAmpFeederSpeed;}), new AmpOpenLoop(amp, () -> -ClimbConstants.kAmpHandoffMaxSpeed), new IntakeOpenLoop(intake, leds, () -> -IntakeConstants.kIntakeSlowSpeed))); 

         new Trigger(() -> {return operatorController.getPOV() == 270;}).whileTrue(new AmpOpenLoop(amp, () -> {return ClimbConstants.kAmpMaxSpeed;}));
        
         new Trigger(() -> {return operatorController.getPOV() == 0;}).whileTrue(new ClimberOpenLoop(climber, () -> {return ClimbConstants.kClimbMaxSpeed;}));
         new Trigger(() -> {return operatorController.getPOV() == 180;}).whileTrue(new ClimberOpenLoop(climber, () -> {return -ClimbConstants.kClimbMaxSpeed;}));

         new Trigger(() -> {return ledController.getStartButton();}).onTrue(new InstantCommand(() -> {
            if(bLedToggle){
                leds.enableLedControl();
                bLedToggle = false;
            } else {
                leds.disableLedControl();
                bLedToggle = true;
            }

        }));

        //new Trigger(() -> {return ledController.getRightBumper();}).onTrue(new InstantCommand(leds::loadNextScheme));
        new Trigger(() -> {return ledController.getRightBumper();}).onTrue(new TransLeds(leds));
        new Trigger(() -> {return ledController.getLeftBumper();}).onTrue(new GayLeds(leds));

    }
    public static boolean getCompressorEnabled(){
        return bCompressorEnabled;
    }

    
}
