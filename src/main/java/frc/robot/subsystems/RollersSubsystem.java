package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RollersSubsystem extends SubsystemBase {
  SparkFlex rollersMotor =
      new SparkFlex(Constants.RollersConstants.rollersMotorCanId, SparkFlex.MotorType.kBrushless);

  public RollersSubsystem() {
    SparkFlexConfig rollersMotorConfig = new SparkFlexConfig();
    rollersMotorConfig.smartCurrentLimit(50);
    rollersMotorConfig.idleMode(IdleMode.kBrake);
    rollersMotor.configure(
        rollersMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setRollersSpeed(double speed) {
    rollersMotor.set(speed);
  }

  public Command autoRollersCommand() {
    return Commands.sequence(
        Commands.waitSeconds(0.5),
        Commands.runOnce(
          () -> setRollersSpeed(Constants.RollersConstants.rollersSpeed)),
        Commands.waitSeconds(5),
        Commands.runOnce(() -> setRollersSpeed(0)));
  }


}