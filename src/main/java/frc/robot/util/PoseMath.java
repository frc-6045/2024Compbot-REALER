package frc.robot.util;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
        if(alliance.get() == Alliance.Red){
            return pose.getTranslation().getDistance(FieldConstants.kRedSpeakerBackLocation.getTranslation());
        } else {
            return pose.getTranslation().getDistance(FieldConstants.kSpeakerBackLocation.getTranslation());
        }
        
    }

    public static Pose2d inverse(Pose2d pose) {
        Rotation2d rotationInverse = pose.getRotation().unaryMinus();
        return new Pose2d(
            pose.getTranslation().unaryMinus().rotateBy(rotationInverse), rotationInverse);
      }

public static Transform2d toTransform2d(Translation2d translation) {
    return new Transform2d(translation, new Rotation2d());
  }

public static Translation2d getAdjustedSpeakerPosition(Pose2d currentPose , ChassisSpeeds robotVel){
    Optional<Alliance> alliance = DriverStation.getAlliance();
    Translation2d goalPose;
    if(alliance.get() == Alliance.Red){
        goalPose = FieldConstants.redCenterSpeakerOpening.toTranslation2d();  
    } else {
        goalPose = FieldConstants.centerSpeakerOpening.toTranslation2d();
    }
        double distanceToSpeaker = currentPose.getTranslation().getDistance(goalPose);
        double x = goalPose.getX() - (robotVel.vxMetersPerSecond * (distanceToSpeaker / 10)); //note 10 is note velocity
        double y = goalPose.getY() - (robotVel.vyMetersPerSecond * (distanceToSpeaker / 10)); //note 10 is note velocity
        Translation2d goalPoseAdjusted = new Translation2d(x, y);
        return goalPoseAdjusted;
}

public static double getTargetAngleRobotToTarget(Pose2d currentPose, ChassisSpeeds robotVel){
    double x = getAdjustedSpeakerPosition(currentPose, robotVel).getX() - currentPose.getX();
    double y = getAdjustedSpeakerPosition(currentPose, robotVel).getY() - currentPose.getY();
    return Math.atan2(y, x);

}
}
