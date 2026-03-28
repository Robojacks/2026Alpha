package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.LimelightHelpers;

public class VisonSubsystem extends SubsystemBase {

  DriveSubsystem driveSubsystem;
  // private static final Translation2d BLUE_HUB   = new Translation2d(4.625594, 4.02);
  private static final double FIELD_LENGTH = 16.54;
  private static final double FIELD_WIDTH = 8.21;

  public VisonSubsystem(DriveSubsystem driveSubsystem) {
    // Constructor code here, if needed
  }

  @Override
  public void periodic() {
    // Reject if spinning too fast
    // if (Math.abs(omegaRps) >= 2.0) return;

    LimelightHelpers.SetRobotOrientation(
        VisionConstants.LEFT, driveSubsystem.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    // LimelightHelpers.SetRobotOrientation(VisionConstants.RIGHT, headingDeg, 0, 0, 0, 0, 0);

    var measurementLeft =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LEFT);
    // var measurementRight =
    // LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.RIGHT);

    boolean rejectLeft = shouldRejectPose(measurementLeft);
    /* boolean rejectRight = shouldRejectPose(measurementRight);

    // Also protect against the null-deref bug in the original code where
    // rejectPoseRight checked llMeasurementLeft instead of llMeasurementRight
    if (measurementRight != null && !rejectRight) {
        if (measurementRight.pose.getX() == 0 && measurementRight.pose.getY() == 0) {
            rejectRight = true;
        }
    }  */

    Rotation2d currentRotation = driveSubsystem.getPose().getRotation();

    /*if (!rejectLeft && !rejectRight) {
        double wLeft  = measurementLeft.avgTagArea;
        double wRight = measurementRight.avgTagArea;
        double x = (measurementLeft.pose.getX()  * wLeft  + measurementRight.pose.getX()  * wRight) / (wLeft + wRight);
        double y = (measurementLeft.pose.getY()  * wLeft  + measurementRight.pose.getY()  * wRight) / (wLeft + wRight);
        double timestamp = Math.min(measurementLeft.timestampSeconds, measurementRight.timestampSeconds);
        addVisionMeasurement(new Pose2d(x, y, currentRotation), timestamp);
    } else */ if (!rejectLeft) {
      double xyStdDev = computeSyStdDev(measurementLeft.avgTagArea, measurementLeft.tagCount);
      addVisionMeasurement(
          new Pose2d(measurementLeft.pose.getX(), measurementLeft.pose.getY(), currentRotation),
          measurementLeft.timestampSeconds,
          VecBuilder.fill(xyStdDev, xyStdDev, 9999999.0));
    } /*else if (!rejectRight) {
          addVisionMeasurement(
              new Pose2d(measurementRight.pose.getX(), measurementRight.pose.getY(), currentRotation),
              measurementRight.timestampSeconds);
      }*/
  }

  /** Returns true if the measurement should be rejected. */
  private boolean shouldRejectPose(LimelightHelpers.PoseEstimate measurement) {
    if (measurement == null) return true;
    if (measurement.tagCount == 0) return true;
    if (measurement.tagCount == 1 && measurement.avgTagArea < 0.05) return true;
    if (measurement.pose.getX() < 0.0 || measurement.pose.getX() > FIELD_LENGTH) return true;
    if (measurement.pose.getY() < 0.0 || measurement.pose.getY() > FIELD_WIDTH) return true;
    if (measurement.pose.getX() == 0 && measurement.pose.getY() == 0) return true;
    return false;
  }

  private double computeSyStdDev(double avgTagArea, int tagCount) {
    // This is a heuristic function that maps the average tag area and tag count to a standard
    // deviation for the y measurement.
    // You may want to tune this function based on your specific use case and testing.
    /* AI suggested code:
    if (tagCount >= 4) {
        return 0.1; // Very confident with 4 or more tags
    } else if (tagCount == 3) {
        return 0.2; // Still pretty good with 3 tags
    } else if (tagCount == 2) {
        return 0.5; // Less confident with only 2 tags
    } else {
        return 1.0; // Default to a high standard deviation for 1 tag, but this case should be rejected by shouldRejectPose
    } */

    // Alexandria code:
    double area = Math.max(avgTagArea, 0.00);
    double base = 0.5 / area;

    // More tags should reduce the standard deviation, but with diminishing returns
    double countFactor = 1.0 / Math.max(tagCount, 1.0);

    return Math.min(base * countFactor, 4.0);
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    driveSubsystem.addVisionMeasurement(
        visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
  }

  // Vision processing methods here, if needed

}
