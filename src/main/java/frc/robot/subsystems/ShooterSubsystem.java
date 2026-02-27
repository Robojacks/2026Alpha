package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  public SparkFlex shooterMotor =
      new SparkFlex(Constants.ShooterConstants.shooterMotorCanId, SparkFlex.MotorType.kBrushless);

  public ShooterSubsystem() {
    SparkFlexConfig shooterMotor1Config = new SparkFlexConfig();
    shooterMotor1Config.smartCurrentLimit(50);
    shooterMotor1Config.idleMode(IdleMode.kBrake);
    shooterMotor.configure(
        shooterMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    // motor1.setIdleMode(SparkMax.IdleMode.kBrake);
  }

  public void setShooterSpeed(double speed) {
    shooterMotor.set(speed);
  }

  public Command autoShooterCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> setShooterSpeed(Constants.ShooterConstants.shooterSpeed)),
        Commands.waitSeconds(5),
        Commands.runOnce(() -> setShooterSpeed(0)));
  }
}
