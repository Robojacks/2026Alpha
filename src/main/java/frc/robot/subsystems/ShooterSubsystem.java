package frc.robot.subsystems;

import com.revrobotics.PersistMode;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  public SparkFlex shooterMotor = new SparkFlex(Constants.ShooterConstants.shooterMotorCanId,
      SparkFlex.MotorType.kBrushless);
  public SparkFlex shooterMotorFallower = new SparkFlex(Constants.ShooterConstants.shooterMotor2CanId,
      SparkFlex.MotorType.kBrushless);
  private PIDController pidController;

  
  public ShooterSubsystem() {
    SparkFlexConfig shooterMotor1Config = new SparkFlexConfig();

    shooterMotor1Config.smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);
    SparkFlexConfig shooterMotor2Config = new SparkFlexConfig();
    shooterMotor2Config.smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake)
        .follow(shooterMotor.getDeviceId(), true);

    shooterMotor.configure(
        shooterMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    // motor1.setIdleMode(SparkMax.IdleMode.kBrake);
    shooterMotorFallower.configure(shooterMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // encoder = shooterMotor.getEncoder();
    pidController = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
    pidController.setTolerance(ShooterConstants.kShooterRpmTolerance);
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
    double output = pidController.calculate(currentRpm, rpm);
    setShooterSpeed(output);
  }

  public Command autoShooterCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> setShooterSpeed(Constants.ShooterConstants.shooterSpeed)),
        Commands.waitSeconds(5),
        Commands.runOnce(() -> setShooterSpeed(0)));
  }

  public Command setShooterDefaultSpeedCommand(double targetRPMs) {
    return Commands.run(
        () -> setPIDSpeed(targetRPMs), this);
  }

  public Command stopShooterCommand() {
    return Commands.runOnce(() -> setShooterSpeed(0), this);
  }
}
