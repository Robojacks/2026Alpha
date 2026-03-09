// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/**
 * Subsystem that controls a shooter motor using a WPILib PID controller to track a target RPM.
 * <p>
 * Exposes methods to set and stop the shooter, query current velocity, and check if the
 * shooter is within tolerance of its setpoint. The PID loop is executed in {@link #periodic()},
 * and telemetry is published to SmartDashboard.
 */
public class PIDsubsystem extends SubsystemBase {
  private final SparkFlex m_leader;
  private final RelativeEncoder m_encoder;
  private final PIDController m_pidController;

  private double m_targetRpm = 0.0;
  private double m_currentSpeed = 0.0;
  private double m_currentPower = 0.0;

  /** Creates a new ShooterSubsystem using WPILib PID velocity control. */
  public PIDsubsystem() {
    m_leader = new SparkFlex(ShooterConstants.kShooterLeaderCanId, MotorType.kBrushless);

    SparkFlexConfig leaderConfig = new SparkFlexConfig();
    leaderConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit((int) ShooterConstants.kShooterCurrentLimitAmps);

    m_leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_encoder = m_leader.getEncoder();
    m_pidController = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
    m_pidController.setTolerance(ShooterConstants.kShooterRpmTolerance);
  }

  /** Set shooter speed in RPM using the WPILib PID controller. */
  public void setRpm(double rpm) {
    m_targetRpm = rpm;
    m_pidController.setSetpoint(rpm);
    SmartDashboard.putNumber("Shooter/TargetRPM", rpm);
  }

  /** Stop the shooter motor. */
  public void stop() {
    m_targetRpm = 0.0;
    m_currentPower = 0.0;
    m_pidController.reset();
    m_leader.stopMotor();
    SmartDashboard.putNumber("Shooter/TargetRPM", 0.0);
  }

  /** Current shooter velocity in RPM. */
  public double getRpm() {
    return m_encoder.getVelocity();
  }

  /** Returns true if shooter is within tolerance of the target. */
  public boolean atSetpoint() {
    return m_pidController.atSetpoint();
  }

  /** Command to run the shooter at a target RPM. */
  public Command runAtRpm(double rpm) {
    return runEnd(() -> setRpm(rpm), this::stop);
  }

  @Override
  public void periodic() {
    if (m_targetRpm < 1.0) {
      m_currentPower = m_targetRpm;
    } else {
      m_currentSpeed = getRpm();
      double powerAdjustment = m_pidController.calculate(m_currentSpeed, m_targetRpm);
      m_currentPower = m_currentPower + powerAdjustment;
      SmartDashboard.putNumber("Shooter/PowerAdjustment", powerAdjustment);
      SmartDashboard.putNumber("Shooter/CurrentSpeed", m_currentSpeed);
    }

    m_leader.set(m_currentPower);
    SmartDashboard.putNumber("Shooter/Output", m_currentPower);

    SmartDashboard.putNumber("Shooter/RPM", getRpm());
    SmartDashboard.putBoolean("Shooter/AtSetpoint", atSetpoint());
  }
}
