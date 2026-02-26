// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class IntakeConstants {
    public static final int intakeMotorCanId = 14;

    public static final double intakeSpeed = -1;
  }

  public static class IntakePivotConstants {
    public static final int intakePivotMotorCanId = 13;

    public static final double intakePivotSpeed = 0.2;
  }

  public static class ShooterConstants {
    public static final int shooterMotorCanId = 11;

    public static final double shooterSpeed = 0.5;
  }

  public static class ShooterFeederConstants {
    public static final int shooterFeederMotorCanId = 12;

    public static final double shooterFeederSpeed = 0.65;
  }

  public static class AgitatorConstants {
    public static final int agitatorMotorCanId = 10;

    public static final double agitatorSpeed = 0.5;
  }

  public static class RollersConstants {
    public static final int rollersMotorCanId = 15;

    public static final double rollersSpeed = 0.5;
  }
}
