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

public class ShooterFeederSubsystem extends SubsystemBase {
  SparkFlex shooterFeederMotor =
      new SparkFlex(
          Constants.ShooterFeederConstants.shooterFeederMotorCanId, SparkFlex.MotorType.kBrushless);

  public ShooterFeederSubsystem() {
    SparkFlexConfig shooterFeederMotorConfig = new SparkFlexConfig();
    shooterFeederMotorConfig.smartCurrentLimit(50);
    shooterFeederMotorConfig.idleMode(IdleMode.kBrake);
    shooterFeederMotor.configure(
        shooterFeederMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setShooterFeederSpeed(double speed) {
    shooterFeederMotor.set(speed);
  }


   public Command autoShooterFeederCommand() {
    return Commands.sequence(
      Commands.waitSeconds(0.5),
      Commands.runOnce(() -> setShooterFeederSpeed(Constants.ShooterFeederConstants.shooterFeederSpeed)),
      Commands.waitSeconds(5),
      Commands.runOnce(() -> setShooterFeederSpeed(0)));

  }



}
