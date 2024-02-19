package frc.robot.util;

import java.util.Optional;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.swerve.DriveSubsystem;

//FIXME: this rn will only work with blue alliance, needs updating for red alliance
public class PoseMath {
    public static double FindShootingAngle(Pose2d pose){ 
        double angle;
        System.out.println(pose.getTranslation());
        double distanceToSpeakerBack = pose.getTranslation().getDistance(FieldConstants.kSpeakerBackLocation.getTranslation());
        double distanceToSpeakerFront = pose.getTranslation().getDistance(FieldConstants.kSpeakerFrontLoation.getTranslation());
        System.out.println(distanceToSpeakerBack + " " + distanceToSpeakerFront);
        double angleToBackLow = Math.toDegrees(Math.atan(FieldConstants.kLowSpeakerOpeningHeight/distanceToSpeakerBack));
        double angleToFrontHigh = Math.toDegrees(Math.atan(FieldConstants.kHighSpeakerOpeningHeight/distanceToSpeakerFront)); 
        System.out.println(angleToBackLow + " " +  angleToFrontHigh);
        angle = (angleToBackLow + angleToFrontHigh) / 2;
        System.out.println("shooting angle: " + angle);
        return angle;
    }

    public static double FindTurningAngle(Pose2d pose){
        double angle;
        double x = FieldConstants.kSpeakerBackLocation.getX() - pose.getX();
        double y = FieldConstants.kSpeakerBackLocation.getY() - pose.getY();
        angle = Math.toDegrees(-Math.atan2(x, y));
        // if(angle < 0){
        //     angle = -angle + 180;
        // }
        return angle;
    }

    public static Rotation2d getTargetAngle(Pose2d point, Pose2d currentPose){
        Rotation2d targetAngle;
        //Rotation2d targetAngle = new Rotation2d(point.getX() - currentPose.getX(), point.getY() - currentPose.getY());
        // if(currentPose.getRotation().getDegrees() < 0){
        //     return targetAngle.minus(targetAngle.times(2));
        // }
        targetAngle = PhotonUtils.getYawToPose(currentPose, point);
        return targetAngle;
    }




    public static double getDistanceToSpeakerBack(Pose2d pose) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isEmpty() && alliance.get() == Alliance.Red){
            return pose.getTranslation().getDistance(FieldConstants.kRedSpeakerBackLocation.getTranslation());
        } else {
            return pose.getTranslation().getDistance(FieldConstants.kSpeakerBackLocation.getTranslation());
        }
        
    }


}
