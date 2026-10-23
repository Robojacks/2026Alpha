package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisonSubsystem extends SubsystemBase {
  private final DriveSubsystem m_driveSubsystem;
  /** Creates a new VisionSubsystem. */
  private final Field2d field2d = new Field2d();


  private final String leftCameraName = "leftCamera";
    private final String rightCameraName = "rightCamera"

  private final PhotonCamera leftCamera;
  private final PhotonCamera rightCamera;

  AprilTagFieldLayout fieldLayout;
  public static Pose2d aprilTag24Pose2d;
  

  private Transform3d leftCameraTransform =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(14), Units.inchesToMeters(11), Units.inchesToMeters(0)),
          new Rotation3d(0, Units.degreesToRadians(10), Units.degreesToRadians(15)));

  private Transform3d rightCameraTransform =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(14), Units.inchesToMeters(-11), Units.inchesToMeters(0)),
          new Rotation3d(0, Units.degreesToRadians(10), Units.degreesToRadians(-15)));


  public VisonSubsystem(DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;

    leftCamera = new PhotonCamera(leftCameraName);
    rightCamera = new PhotonCamera(rightCameraName);

    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    aprilTag24Pose2d = fieldLayout.getTagPose(24).get().toPose2d();
  }

  @Override
  public void periodic() {
    updateCameras();
  }

  public void updateCameras() {
    updateCamera(leftCamera, leftCameraTransform);
    updateCamera(rightCamera, rightCameraTransform);
    SmartDashboard.putData("Vision/Field", field2d);
    SmartDashboard.putBoolean("Vision/LeftConnected", leftCamera.isConnected());
    SmartDashboard.putBoolean("Vision/RightConnected", rightCamera.isConnected());
  }

  private void updateCamera(PhotonCamera camera, Transform3d cameraTransform) {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    for (var result : results) {
      if (result.hasTargets()) {
        PhotonTrackedTarget tar = result.getBestTarget();

        if (tar != null) {
          Transform3d c2t = tar.getBestCameraToTarget();

          Optional<Pose3d> tagPose = fieldLayout.getTagPose(tar.getFiducialId());

          if (tagPose.isEmpty() || tar.getPoseAmbiguity() > 0.15) {
            continue;
          }

          // Get current robot pose by comparing camera pose to tag pose and transforming with
          // camera transform.
          Pose2d p =
              PhotonUtils.estimateFieldToRobotAprilTag(
                      c2t, tagPose.get(), cameraTransform.inverse())
                  .toPose2d();

          // Get distance from robot to april tag
          Pose2d tag = tagPose.get().toPose2d();
          double distance = PhotonUtils.getDistanceToPose(p, tag);

          // Dynamically scale trust with range and ambiguity.
          // Lower std dev = more trust from the estimator.
          double ambiguity = Math.max(0.0, tar.getPoseAmbiguity());
          double xyStdDev = calculateXYStdDev(distance, ambiguity);
          double thetaStdDev = calculateThetaStdDev(distance, ambiguity);

          // Update drive subsystem with vision measurement, including standard deviation for Kalman
          // filter.
          m_driveSubsystem.addVisionMeasurement(
              p,
              result.getTimestampSeconds(),
              VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));

          field2d.setRobotPose(p);
          SmartDashboard.putNumber("Vision/DistanceMeters", distance);
          SmartDashboard.putNumber("Vision/Ambiguity", ambiguity);
          SmartDashboard.putNumber("Vision/XYStdDev", xyStdDev);
          SmartDashboard.putNumber("Vision/ThetaStdDev", thetaStdDev);
        }
      }
    }
  }

  private double calculateXYStdDev(double distanceMeters, double ambiguity) {
    double std = 0.05 + (0.02 * distanceMeters * distanceMeters) + (0.35 * ambiguity);
    return Math.max(0.05, Math.min(2.0, std));
  }

  private double calculateThetaStdDev(double distanceMeters, double ambiguity) {
    double std = 0.1 + (0.04 * distanceMeters * distanceMeters) + (0.8 * ambiguity);
    return Math.max(0.1, Math.min(4.0, std));
  }

  public void dummyMethod() {
    // Intentionally empty: keeps this subsystem referenced in RobotContainer.
  }
}