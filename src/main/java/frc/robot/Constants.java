// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {

  public static class IntakeConstants {
    public static final int intakeMotorCanId = 14;

    public static final double intakeSpeed = -1;
  }

  public static class IntakePivotConstants {
    public static final int intakePivotMotorCanId = 13;
    public static final int intakePivotMotorCurrentLimitAmps = 25;

    public static final double intakePivotSpeed = -0.2;
  }

  public static class ShooterConstants {
    public static final int shooterMotorCanId = 11;

    public static final int shooterMotor2CanId = 16;

    public static final double shooterSpeedPercent = 0.45; // .45 changed by william
    public static final int shooterTargetRpm = 2800;
    public static final int shooterIdleRpm = 1000;
    public static final double kP = 0.00045;
    public static final double kI = 0.000;
    public static final double kD = 0.0000;

    public static final double kS = 0.1;
    public static final double kV = 0.0001;
    public static final double kA = 0.00001;

    public static final double kShooterRpmTolerance = 60;
  }

  public static class ShooterFeederConstants {
    public static final int shooterFeederMotorCanId = 12;

    public static final double shooterFeederSpeed = .75;
  }

  public static class AgitatorConstants {
    public static final int agitatorMotorCanId = 10;

    public static final double agitatorSpeed = 0.5;
  }

  public static class RollersConstants {
    public static final int rollersMotorCanId = 15;

    public static final double rollersSpeed = 1;
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK FLEX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 2;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearRightDrivingCanId = 3;

    public static final int kFrontLeftTurningCanId = 5;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kFrontRightTurningCanId = 8;
    public static final int kRearRightTurningCanId = 7;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.1;
    public static final int kOperatorControllerPort = 1;
    public static final double kOperatorDeadband = 0.1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // PathPlanner HolonomicDriveController PID constants
    public static final double kPPTranslationP = 1.0;
    public static final double kPPRotationP = 1.0;

    // PathPlanner robot config
    public static final double kRobotMassKg = Units.lbsToKilograms(40); // converted to kg
    public static final double kRobotMOI = 6.883;
    public static final double kWheelRadiusMeters = ModuleConstants.kWheelDiameterMeters / 2.0;
    public static final double kMaxDriveSpeedMetersPerSec = DriveConstants.kMaxSpeedMetersPerSecond;
    public static final double kDriveCurrentLimitAmps = 50.0;
    public static final double kWheelCOF = 1.2;

    public static final Translation2d[] kModuleTranslations =
        new Translation2d[] {
          new Translation2d(DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2),
          new Translation2d(DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2),
          new Translation2d(-DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2),
          new Translation2d(-DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2)
        };

    public static final RobotConfig kPPConfig =
        new RobotConfig(
            kRobotMassKg,
            kRobotMOI,
            new ModuleConfig(
                kWheelRadiusMeters,
                kMaxDriveSpeedMetersPerSec,
                kWheelCOF,
                DCMotor.getNEO(1).withReduction(ModuleConstants.kDrivingMotorReduction),
                kDriveCurrentLimitAmps,
                1),
            kModuleTranslations);

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class VisionConstants {

    public static final String LIMELIGHT_NAME = "limelight-swerve";

    public static final double MountPostitionForward = Units.inchesToMeters(2.5);
    public static final double MountPositionSide = 0.0;
    public static final double MountPositionUp = Units.inchesToMeters(20.5);
    public static final double MountPositionRoll = 0;
    public static final double MountPositionPitch = 30;
    public static final double MountPositionYaw = 0;

    public static final Transform3d ROBOT_TO_CAMERA =
        new Transform3d(
            new Translation3d(.18415, .320675, 0.1666875),
            new Rotation3d(Math.toRadians(180), Math.toRadians(28.44), 0));

    public static final Vector<N3> SINGLE_TAG_STD_DEVS = VecBuilder.fill(1.0, 1.0, 2.0);
    public static final Vector<N3> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1.0);

    public static final int TARGET_TAG_ID = 4;
    public static final double FOLLOW_DISTANCE_METERS = 1.5;
    public static final double FOLLOW_SPEED = 0.5;

    public static final double FOLLOW_KP_TRANSLATION = 0.2;
    public static final double FOLLOW_KP_ROTATION = 0.02;

    public static final double DISTANCE_TOLERANCE = 0.1;
    public static final double ANGLE_TOLERANCE = 2.0;
    public static final String RIGHT = "limelight-right";
  }
}
