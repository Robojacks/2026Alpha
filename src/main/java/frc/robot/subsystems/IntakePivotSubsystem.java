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
  // double targetPosition = 0.0;
  // private final AbsoluteEncoder encoder;
  // private PIDController pidController = new PIDController(0.3, 0, .05);

  SparkMax intakePivotMotor =
      new SparkMax(
          Constants.IntakePivotConstants.intakePivotMotorCanId, SparkMax.MotorType.kBrushless);

  public IntakePivotSubsystem() {
    // pidController.setTolerance(0.01);
    SparkMaxConfig intakePivotMotorConfig = new SparkMaxConfig();
    intakePivotMotorConfig.smartCurrentLimit(80);
    intakePivotMotorConfig.idleMode(IdleMode.kCoast);
    intakePivotMotor.configure(
        intakePivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    // Initialize the absolute encoder attached to the pivot motor
    // encoder = intakePivotMotor.getAbsoluteEncoder();
  }

  public void setIntakePivotSpeed(double speed) {
    intakePivotMotor.set(speed);
  }

  public Command autoIntakePivotDownCommand() {
    return Commands.sequence(
        Commands.runOnce(
            () -> setIntakePivotSpeed(Constants.IntakePivotConstants.intakePivotSpeed)),
        Commands.waitSeconds(.7),
        Commands.runOnce(() -> setIntakePivotSpeed(0)));
  }

  public Command autoIntakePivotUpCommand() {
    return Commands.sequence(
        Commands.runOnce(
            () -> setIntakePivotSpeed(-Constants.IntakePivotConstants.intakePivotSpeed)),
        Commands.waitSeconds(1.15),
        Commands.runOnce(() -> setIntakePivotSpeed(0)));
  }

  /**
   * Returns a simple command that sets the pivot target to OUT immediately. Use this for button
   * bindings or composing in command groups.
   */
  // public Command setOutCommand() {
  //   return Commands.runOnce(() -> setOut(), this);
}

  /**
   * Returns a simple command that sets the pivot target to STOW immediately. Use this for button
   * bindings or composing in command groups.
   */
  // public Command setStowCommand() {
  //   return Commands.runOnce(() -> setStow(), this);
  // }

  // private static enum IntakePivotPosition {
  //   Stow(.28),
  //   OUT(0.001);

  //   private final double position;

  //   IntakePivotPosition(double position) {
  //     this.position = position;
  //   }
  // }

  // @Override
  // public void periodic() {

    // if (encoder == null) {
    //   SmartDashboard.putString("Intake Pivot Encoder Status", "MISSING");
    //   return;
    // }

    // double pos = encoder.getPosition();
    // SmartDashboard.putNumber("Intake Pivot Encoder Position", pos);
    // SmartDashboard.putNumber("Intake Pivot Target Position", targetPosition);
    // SmartDashboard.putBoolean("At Target", Math.abs(pos - targetPosition) < 0.01);
    // pidController.setSetpoint(targetPosition);
    // double speed = pidController.calculate(pos);

  //   intakePivotMotor.set(-speed);
  // }

  // public void setOut() {
  //   targetPosition = IntakePivotPosition.OUT.position;
  // }

  // public void setStow() {
  //   targetPosition = IntakePivotPosition.Stow.position;
  // }
