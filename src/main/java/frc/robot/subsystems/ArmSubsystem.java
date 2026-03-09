package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

  private static final int rotateMotorCANId = 11;
  private SparkMax rotateMotor;
  private AbsoluteEncoder absoluteEncoder;
  private PIDController pidController;

  double targetPosition = .5;

  public ArmSubsystem() {
    rotateMotor = new SparkMax(rotateMotorCANId, MotorType.kBrushless);
    absoluteEncoder = rotateMotor.getAbsoluteEncoder();

    pidController = new PIDController(1, 0, 0);
    pidController.enableContinuousInput(0, 1);
  }

  @Override
  public void periodic() {
    pidController.setSetpoint(targetPosition);
    double speed = pidController.calculate(absoluteEncoder.getPosition());

    rotateMotor.set(-speed);
  }

  public void setOut() {
    targetPosition = .5;
  }

  public void setOff() {
    targetPosition = .5;
  }
}
