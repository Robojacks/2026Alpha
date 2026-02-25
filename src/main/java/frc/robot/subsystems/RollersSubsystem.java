package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
    
}
