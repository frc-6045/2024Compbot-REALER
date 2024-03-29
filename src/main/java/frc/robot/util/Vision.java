// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.swerve.DriveSubsystem;


/** Add your docs here. */
public class Vision {
  private static PhotonPoseEstimator m_visionPoseEstimator;
  private static AprilTagFieldLayout fieldLayout;
  private static Field2d m_photonVisionField = new Field2d();
  private final DriveSubsystem m_drive;
  private Pose2d visionPose;
  private static double displacementToTargetAngle = 0;
  public Vision(DriveSubsystem drive){
  fieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
  if(FieldConstants.kVisionEnable){
    try {
      m_visionPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new PhotonCamera("Arducam_OV2311_USB_Camera"), new Transform3d(new Translation3d(-Units.inchesToMeters(5.75), Units.inchesToMeters(13.75), Units.inchesToMeters(18.5)), new Rotation3d(0, Units.degreesToRadians(21),Units.degreesToRadians(180)))); //TODO: -5.75, 13.75, 18.5 /-4.5, 12.7, 17
    } catch(IOException e){
      System.out.println(e.getMessage() + "\n april tags didnt load");
    }
  }
    m_drive = drive;
  }

  public static PhotonPoseEstimator getPoseEstimator(){
    return m_visionPoseEstimator;
  }
  
  // All of this is stolen from Isaac, thanks Isaac!!! (feel free to refactor at any time)
  public static void addFilteredPoseData(Pose2d currentPose, SwerveDrivePoseEstimator m_poseEstimator) {
    PhotonPoseEstimator poseEstimator = Vision.getPoseEstimator();
      // print out the time for this line to run 
      Optional<EstimatedRobotPose> pose = poseEstimator.update();
      if(!DriverStation.isAutonomousEnabled()){
      if (pose.isPresent()) {
        Pose3d pose3d = pose.get().estimatedPose;
        Pose2d pose2d = pose3d.toPose2d();
        if (
            pose3d.getX() >= -FieldConstants.VISION_FIELD_MARGIN &&
            pose3d.getX() <= FieldConstants.FIELD_LENGTH + FieldConstants.VISION_FIELD_MARGIN &&
            pose3d.getY() >= -FieldConstants.VISION_FIELD_MARGIN &&
            pose3d.getY() <= FieldConstants.FIELD_WIDTH + FieldConstants.VISION_FIELD_MARGIN &&
            pose3d.getZ() >= -FieldConstants.VISION_Z_MARGIN &&
            pose3d.getZ() <= FieldConstants.VISION_Z_MARGIN
           ) {
              double sum = 0.0;
              for (PhotonTrackedTarget target : pose.get().targetsUsed) {
                  Optional<Pose3d> tagPose =
                      fieldLayout.getTagPose(target.getFiducialId());
                  if (tagPose.isEmpty()) continue;
                  sum += currentPose.getTranslation().getDistance(tagPose.get().getTranslation().toTranslation2d());
              }

              int tagCount = pose.get().targetsUsed.size();
              double stdScale = Math.pow(sum / tagCount, 2.0) / tagCount;
              double xyStd = FieldConstants.VISION_STD_XY_SCALE * stdScale;
              double rotStd = FieldConstants.VISION_STD_ROT_SCALE * stdScale;
              //time this as well
              m_poseEstimator.addVisionMeasurement(pose2d, pose.get().timestampSeconds, VecBuilder.fill(xyStd, xyStd, rotStd));
          }

          m_photonVisionField.setRobotPose(pose2d);
      }
    }
  }

  public void UpdateVision() {
  if(FieldConstants.kVisionEnable && !DriverStation.isAutonomousEnabled()){
     m_visionPoseEstimator.update().ifPresent(estimatedRobotPose -> {
      //System.out.println(estimatedRobotPose.estimatedPose.toPose2d().toString());
      
      m_drive.getPoseEstimator().addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
      visionPose = estimatedRobotPose.estimatedPose.toPose2d();
      //SmartDashboard.putNumber("Estimated Angle", PoseMath.FindShootingAngle(estimatedRobotPose.estimatedPose.toPose2d())); //enhnngg
      SmartDashboard.putNumber("estimated dist", PoseMath.getDistanceToSpeakerBack(estimatedRobotPose.estimatedPose.toPose2d()));
      SmartDashboard.putNumber("vision angle", visionPose.getRotation().getDegrees());
      //SmartDashboard.putNumber("estimated proper turning angle", PoseMath.getTargetAngle(FieldConstants.kSpeakerBackLocation, estimatedRobotPose.estimatedPose.toPose2d()).getDegrees());
      SmartDashboard.putNumber("lookup table number", LookupTables.getAngleTable().get(PoseMath.getDistanceToSpeakerBack(estimatedRobotPose.estimatedPose.toPose2d())));
    });
  }
  }

  public Pose2d getVisionPose() {
    return visionPose;
  }

  public static void setDisplacementToTargetAngle(double displacement){
    displacementToTargetAngle = displacement;
  }
}
