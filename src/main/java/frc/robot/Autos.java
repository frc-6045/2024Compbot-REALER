package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class Autos {

    //FIXME: just rewrite all of this terribleness
    private final DriveSubsystem m_Drivetrain;
    private final Feeder m_Feeder;
    private final Intake m_Intake;
    private final Pneumatics m_Pneumatics;
    private final Shooter m_Shooter;
    private SendableChooser<Command> autoChooser;
    private ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
    public Autos(DriveSubsystem drive, Feeder feed, Intake intake, Pneumatics pneumatic, Shooter shooter){
        m_Drivetrain = drive;
        m_Feeder = feed;
        m_Intake = intake;
        m_Pneumatics = pneumatic;
        m_Shooter = shooter;
        autoChooser = new SendableChooser<Command>();
    
        // autoChooser = AutoBuilder.buildAutoChooser(); 
        
        autoChooser.addOption("DoNothing", new InstantCommand());
        autoChooser.addOption("Drivetrain Characterization", new FeedForwardCharacterization(m_Drivetrain, true, new FeedForwardCharacterizationData("DriveSubsystem"), 
            m_Drivetrain::runCharacterizationVolts, m_Drivetrain::getCharacterizationVelocity));
        autoChooser.addOption("Shooter Characterization", new FeedForwardCharacterization(m_Shooter, true, new FeedForwardCharacterizationData("Shooter"),
                m_Shooter::runCharacterizationVolts , m_Shooter::getCharacterizationVelocity));
        
        // autoChooser.addOption("FarSideSteal2ndShot Blue", AutoBuilder.buildAuto("FarSideSteal2ndShot"));
        // autoChooser.addOption("FarSideSteal2ndShot Red", AutoBuilder.buildAuto("FarSideSteal2ndShot Red"));
        // autoChooser.addOption("FarSideSteal2ndShot3Note Blue", AutoBuilder.buildAuto("FarSideSteal2ndShot3Note"));
        // autoChooser.addOption("FarSideSteal2ndShot3Note Red", AutoBuilder.buildAuto("FarSideSteal2ndShot3Note Red"));
        autoChooser.addOption("GCR Finals", AutoBuilder.buildAuto("GCRFinalsAuto"));
        // autoChooser.addOption("NOMOVE", AutoBuilder.buildAuto("NOMOVE"));
        autoChooser.addOption("FarSide2Piece1stNote", AutoBuilder.buildAuto("FarSide2Piece"));
        autoChooser.addOption("FarSideBluePiece1stNote", AutoBuilder.buildAuto("FarSideBluePiece"));
        autoChooser.addOption("FarSide2Piece2ndNote", AutoBuilder.buildAuto("FarSide2Piece2ndNote"));
        autoChooser.addOption("Center4Note", AutoBuilder.buildAuto("Center4Note"));
        // autoChooser.addOption("FarSideSteal", AutoBuilder.buildAuto("FarSideSteal"));
        // autoChooser.addOption("3RingMidfieldAlt", AutoBuilder.buildAuto("3RingMidfieldAlt"));
        // autoChooser.addOption("Basic Test Auto", AutoBuilder.buildAuto("BasicTestAuto"));
        // autoChooser.addOption("3RingMidfield", AutoBuilder.buildAuto("3RingMidfield"));
        // autoChooser.addOption("4RingRed", AutoBuilder.buildAuto("4RingRed"));
        // autoChooser.addOption("4RingBlue", AutoBuilder.buildAuto("4RingBlue"));
        // autoChooser.addOption("3RingNoPodium", AutoBuilder.buildAuto("3RingNoPodium"));
        // autoChooser.addOption("3RingNoTop", AutoBuilder.buildAuto("3RingNoTop"));
        // autoChooser.addOption("CenterSteal", AutoBuilder.buildAuto("CenterSteal"));
        autoChooser.addOption("ShootAndDoNothing", AutoBuilder.buildAuto("ShootAndDoNothing"));
        // autoChooser.addOption("ShootASndDoNothingLeft", AutoBuilder.buildAuto("ShootAndDoNothingLeft"));
        // autoChooser.addOption("ShootAndDoNothingRight", AutoBuilder.buildAuto("ShootAndDoNothingRight"));
        // autoChooser.addOption("ShootAndOutRight", AutoBuilder.buildAuto("ShootAndOutRight"));
        // autoChooser.addOption("DoNothing", AutoBuilder.buildAuto("DoNothing"));
        // autoChooser.addOption("DoNothingLeft", AutoBuilder.buildAuto("DoNothingLeft"));
        // autoChooser.addOption("DoNothingRight", AutoBuilder.buildAuto("DoNothingRight"));
        autoChooser.addOption("openSide1Piece", AutoBuilder.buildAuto("openSide1Piece"));


        SmartDashboard.putData("Autos", autoChooser);
    }
    
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
