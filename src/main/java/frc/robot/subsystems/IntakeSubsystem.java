package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  SparkMax intakeMotor =
      new SparkMax(Constants.IntakeConstants.intakeMotorCanId, SparkMax.MotorType.kBrushless);

  public IntakeSubsystem() {
    SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    intakeMotorConfig.smartCurrentLimit(50);
    intakeMotorConfig.idleMode(IdleMode.kBrake);
    intakeMotor.configure(
        intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }


   public Command autoIntakeCommand() {
    return Commands.sequence(
      Commands.runOnce(() -> setIntakeSpeed(Constants.IntakeConstants.intakeSpeed)),
      Commands.waitSeconds(5),
      Commands.runOnce(() -> setIntakeSpeed(0)));

  }




}
