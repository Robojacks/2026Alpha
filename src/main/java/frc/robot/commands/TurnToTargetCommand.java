package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.Objects;
import java.util.function.Supplier;

/**
 * Command that turns the robot to face a field-relative target position using the DriveSubsystem's
 * pose estimator.
 *
 * <p>Usage: new TurnToTargetCommand(driveSubsystem, () -> targetTranslation) or new
 * TurnToTargetCommand(driveSubsystem, targetTranslation)
 */
public class TurnToTargetCommand extends Command {
  private final DriveSubsystem m_drive;
  private final Supplier<Translation2d> m_targetSupplier;
  private final PIDController m_rotationPid;
  // private final Translation2d;

  /** Create a command that turns to a dynamic target supplied each loop. */
  public TurnToTargetCommand(DriveSubsystem drive, Supplier<Translation2d> targetSupplier) {
    m_drive = Objects.requireNonNull(drive);
    m_targetSupplier = Objects.requireNonNull(targetSupplier);
    m_rotationPid = new PIDController(AutoConstants.kPThetaController, 0.0, 0.0);
    m_rotationPid.enableContinuousInput(-Math.PI, Math.PI);
    m_rotationPid.setTolerance(Math.toRadians(2.0)); // 2 degrees tolerance

    addRequirements(m_drive);
  }

  /** Create a command that turns to a fixed target. */
  public TurnToTargetCommand(DriveSubsystem drive, Translation2d fixedTarget) {
    this(drive, () -> fixedTarget);
  }

  @Override
  public void initialize() {
    // Nothing to do
  }

  @Override
  public void execute() {
    Pose2d robotPose = m_drive.getPose();
    Translation2d robotPos = robotPose.getTranslation();
    Translation2d target = m_targetSupplier.get();
    if (target == null) {
      // No target available; do nothing
      m_drive.drive(0, 0, 0, false);
      return;
    }

    double dx = target.getX() - robotPos.getX();
    double dy = target.getY() - robotPos.getY();

    double desiredHeading = Math.atan2(dy, dx);

    double currentHeading = robotPose.getRotation().getRadians();

    // PID: note calculate(measurement, setpoint)
    double rotRadPerSec = m_rotationPid.calculate(currentHeading, desiredHeading);

    // Convert to normalized rotation input expected by DriveSubsystem.drive()
    double rotNormalized = rotRadPerSec / DriveConstants.kMaxAngularSpeed;
    rotNormalized = MathUtil.clamp(rotNormalized, -1.0, 1.0);

    // Command rotation only (no translation)
    m_drive.drive(0.0, 0.0, rotNormalized, false);
    SmartDashboard.putNumber(
        "TurnToTargetCommand/desiredHeadingDeg", Math.toDegrees(desiredHeading));
  }

  @Override
  public void end(boolean interrupted) {
    // Stop rotation when finished or interrupted
    m_drive.drive(0.0, 0.0, 0.0, false);
  }

  @Override
  public boolean isFinished() {
    return m_rotationPid.atSetpoint();
  }
}
