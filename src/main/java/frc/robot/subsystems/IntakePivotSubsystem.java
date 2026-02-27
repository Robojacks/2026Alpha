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

public class IntakePivotSubsystem extends SubsystemBase {
  SparkMax intakePivotMotor =
      new SparkMax(
          Constants.IntakePivotConstants.intakePivotMotorCanId, SparkMax.MotorType.kBrushless);

  public IntakePivotSubsystem() {
    SparkMaxConfig intakePivotMotorConfig = new SparkMaxConfig();
    intakePivotMotorConfig.smartCurrentLimit(50);
    intakePivotMotorConfig.idleMode(IdleMode.kBrake);
    intakePivotMotor.configure(
        intakePivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setIntakePivotSpeed(double speed) {
    intakePivotMotor.set(speed);
  }

  public Command autoIntakePivotUpCommand() {
    return Commands.sequence(
        Commands.runOnce(
            () -> setIntakePivotSpeed(Constants.IntakePivotConstants.intakePivotSpeed)),
        Commands.waitSeconds(.5),
        Commands.runOnce(() -> setIntakePivotSpeed(0)));
  }

  public Command autoIntakePivotDownCommand() {
    return Commands.sequence(
        Commands.runOnce(
            () -> setIntakePivotSpeed(-Constants.IntakePivotConstants.intakePivotSpeed)),
        Commands.waitSeconds(.5),
        Commands.runOnce(() -> setIntakePivotSpeed(0)));
  }
}
