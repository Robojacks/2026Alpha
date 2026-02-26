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

public class AgitatorSubsystem extends SubsystemBase {
  SparkMax agitatorMotor =
      new SparkMax(Constants.AgitatorConstants.agitatorMotorCanId, SparkMax.MotorType.kBrushless);

  public AgitatorSubsystem() {
    SparkMaxConfig agitatorMotorConfig = new SparkMaxConfig();
    agitatorMotorConfig.smartCurrentLimit(50);
    agitatorMotorConfig.idleMode(IdleMode.kBrake);
    agitatorMotor.configure(
        agitatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setAgitatorSpeed(double speed) {
    agitatorMotor.set(speed);
  }

  

  public Command autoAgitatorCommand() {
    return Commands.sequence(
      Commands.waitSeconds(0.5),
      Commands.runOnce(() -> setAgitatorSpeed(Constants.AgitatorConstants.agitatorSpeed)),
      Commands.waitSeconds(5),
      Commands.runOnce(() -> setAgitatorSpeed(0)));


  }

}
