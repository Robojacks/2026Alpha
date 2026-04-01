package frc.robot.subsystems;

import com.revrobotics.PersistMode;
// import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  public SparkFlex shooterMotor =
      new SparkFlex(Constants.ShooterConstants.shooterMotorCanId, SparkFlex.MotorType.kBrushless);

  public SparkFlex shooterMotorFallower =
      new SparkFlex(Constants.ShooterConstants.shooterMotor2CanId, SparkFlex.MotorType.kBrushless);

  private PIDController pidController;
  private final SimpleMotorFeedforward feedforward;

  public ShooterSubsystem() {
    SparkFlexConfig shooterMotor1Config = new SparkFlexConfig();

    shooterMotor1Config.smartCurrentLimit(50).idleMode(IdleMode.kBrake);

    SparkFlexConfig shooterMotor2Config = new SparkFlexConfig();
    shooterMotor2Config
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake)
        .follow(shooterMotor.getDeviceId(), true);

    shooterMotor.configure(
        shooterMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    // motor1.setIdleMode(SparkMax.IdleMode.kBrake);

    shooterMotorFallower.configure(
        shooterMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // encoder = shooterMotor.getEncoder();

    pidController =
        new PIDController(
            SmartDashboard.getNumber("Shooter/kP", ShooterConstants.kP),
            SmartDashboard.getNumber("Shooter/kI", ShooterConstants.kI),
            SmartDashboard.getNumber("Shooter/kD", ShooterConstants.kD));
    pidController.setTolerance(ShooterConstants.kShooterRpmTolerance);
    SmartDashboard.putData("pid", pidController);

    SmartDashboard.putNumber("kS", ShooterConstants.kS);
    SmartDashboard.putNumber("kV", ShooterConstants.kV);
    SmartDashboard.putNumber("kA", ShooterConstants.kA);
    // Feedforward
    feedforward =
        new SimpleMotorFeedforward(
            SmartDashboard.getNumber("kS", ShooterConstants.kS),
            SmartDashboard.getNumber("kV", ShooterConstants.kV),
            SmartDashboard.getNumber("kA", ShooterConstants.kA));

    /*this.setDefaultCommand(
    setShooterDefaultSpeedCommand(
        2800)); // Set a default target RPM for the shooter when no other commands are running*/
  }

  public void setShooterSpeed(double speed) {
    shooterMotor.set(speed);
  }

  /*
   * public void setRPMs(double output) {
   * // This method would convert the PID controller output to a motor speed and
   * set it.
   * // The conversion would depend on the characteristics of your motor and
   * shooter mechanism.
   * // For example, you might need to scale the output to fit within the motor's
   * input range.
   * double scaledOutput = output; // You may need to apply scaling here based on
   * your system's requirements.
   * shooterMotor.set(scaledOutput);
   * }
   */

  public void setPIDSpeed(double rpm) {
    // This method would use the PID controller to set the shooter speed based on
    // the target RPM.
    // The actual implementation would depend on how you want to integrate the PID
    // controller with the motor output.
    double currentRpm = shooterMotor.getEncoder().getVelocity();
    // Calculate the output from the PID controller based on the current RPM and
    // target RPM.
    double pidSpeed = pidController.calculate(currentRpm, rpm);

    double ffspeed = feedforward.calculate(rpm);
    setShooterSpeed(pidSpeed + ffspeed);
  }

  public Command autoShooterCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> setShooterSpeed(Constants.ShooterConstants.shooterSpeedPercent)),
        Commands.waitSeconds(5),
        Commands.runOnce(() -> setShooterSpeed(0)));
  }

  public Command autoShooterLongCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> setShooterSpeed(Constants.ShooterConstants.shooterSpeedPercent)),
        Commands.waitSeconds(10),
        Commands.runOnce(() -> setShooterSpeed(0)));
  }

  public Command setShooterDefaultSpeedCommand(double targetRPMs) {
    return Commands.run(() -> setPIDSpeed(targetRPMs), this);
  }

  public Command stopShooterCommand() {
    return Commands.runOnce(() -> setShooterSpeed(0), this);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter/CurrentRPM", shooterMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter/TargetRPM", pidController.getSetpoint());
    SmartDashboard.putBoolean("Shooter/AtSetpoint", pidController.atSetpoint());
  }
}
