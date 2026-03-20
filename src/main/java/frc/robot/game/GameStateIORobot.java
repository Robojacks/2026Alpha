// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.game;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.game.GameState.GamePhase;

/**
 * Implements {@link GameStateIO} using WPILib {@link DriverStation} state to derive alliance,
 * phase timing, and shift activity for match logic. Tracks the first active alliance from the
 * game-specific message, determines the current {@link GamePhase}, and exposes helper methods
 * to compute warnings and timing until or during an alliance's active shift.
 * <p>
 * This class updates {@link GameStateInputs} with:
 * <ul>
 *   <li>Current alliance and first active alliance</li>
 *   <li>Current game phase</li>
 *   <li>Shift warnings (head back, green light)</li>
 *   <li>Match type</li>
 * </ul>
 * It also provides utility methods to evaluate whether the robot's alliance is active,
 * determine the currently active alliance, and compute time remaining or until the next shift.
 */
public class GameStateIORobot implements GameStateIO {
  private Alliance alliance = Alliance.Blue;
  private Alliance firstActiveAlliance;
  private boolean receivedGameMessage = false;
  private GamePhase currentPhase = GamePhase.PRE_MATCH;

  public GameStateIORobot() {}

  @Override
  public void updateInputs(GameStateInputs inputs) {
    updateAlliance();
    updateFirstActiveAlliance();
    updateGamePhase();
    inputs.alliance = alliance;
    inputs.firstActiveAlliance = firstActiveAlliance;
    inputs.phase = currentPhase;
    inputs.shouldHeadBack = isHeadBackWarning();
    inputs.shouldStartShooting = isGreenLightPreShift();
    inputs.matchType = DriverStation.getMatchType();
  }

  private void updateAlliance() {
    alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  public void updateFirstActiveAlliance() {
    if (receivedGameMessage) return;
    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData != null && gameData.length() > 0) {
      char c = gameData.charAt(0);
      if (c == 'B') {
        firstActiveAlliance = Alliance.Blue;
        receivedGameMessage = true;
      } else if (c == 'R') {
        firstActiveAlliance = Alliance.Red;
        receivedGameMessage = true;
      }
    }
  }

/**
 * Updates the {@link #currentPhase} based on the Driver Station mode and match time.
 * <ul>
 *   <li>If autonomous: sets {@code AUTO}.</li>
 *   <li>If teleop: maps remaining match time to phase thresholds
 *       (TRANSITION, SHIFT_1–SHIFT_4, END_GAME, POST_MATCH).</li>
 *   <li>If disabled: keeps {@code POST_MATCH} after end game; otherwise sets {@code PRE_MATCH}.</li>
 * </ul>
 */
  private void updateGamePhase() {
    if (DriverStation.isAutonomous()) {
      currentPhase = GamePhase.AUTO;
    } else if (DriverStation.isTeleop()) {
      double t = DriverStation.getMatchTime();
      if (t > 130) currentPhase = GamePhase.TRANSITION;
      else if (t > 105) currentPhase = GamePhase.SHIFT_1;
      else if (t > 80) currentPhase = GamePhase.SHIFT_2;
      else if (t > 55) currentPhase = GamePhase.SHIFT_3;
      else if (t > 30) currentPhase = GamePhase.SHIFT_4;
      else if (t > 0) currentPhase = GamePhase.END_GAME;
      else currentPhase = GamePhase.POST_MATCH;
    } else {
      // Disabled
      if (currentPhase == GamePhase.END_GAME || currentPhase == GamePhase.POST_MATCH) {
        currentPhase = GamePhase.POST_MATCH;
      } else if (currentPhase != GamePhase.POST_MATCH) {
        currentPhase = GamePhase.PRE_MATCH;
      }
    }
  }

  public boolean isOurAllianceActive() {
    Alliance active = getCurrentlyActiveAlliance();
    return active == null || alliance == active;
  }

  public Alliance getCurrentlyActiveAlliance() {
    if (!receivedGameMessage || firstActiveAlliance == null) return null;

    Alliance other = (firstActiveAlliance == Alliance.Blue) ? Alliance.Red : Alliance.Blue;

    switch (currentPhase) {
      case SHIFT_1:
      case SHIFT_3:
        return firstActiveAlliance;
      case SHIFT_2:
      case SHIFT_4:
        return other;
      default:
        return null; // Both active (auto, transition, endgame)
    }
  }

  /** Seconds until our next active shift starts. 0 if already active. */
  public double getSecondsUntilOurNextShift() {
    if (!receivedGameMessage || firstActiveAlliance == null || isOurAllianceActive()) return 0;

    double t = DriverStation.getMatchTime();
    boolean weAreFirst = (alliance == firstActiveAlliance);

    switch (currentPhase) {
      case TRANSITION:
        return weAreFirst ? Math.max(0, t - 130) : Math.max(0, t - 105);
      case SHIFT_1:
        return weAreFirst ? 0 : Math.max(0, t - 105);
      case SHIFT_2:
        return weAreFirst ? Math.max(0, t - 80) : 0;
      case SHIFT_3:
        return weAreFirst ? 0 : Math.max(0, t - 55);
      case SHIFT_4:
        return weAreFirst ? Math.max(0, t - 30) : 0;
      default:
        return 0;
    }
  }

  /** True when 5-3 seconds before our shift. Drivers should head to scoring position. */
  @Override
  public boolean isHeadBackWarning() {
    if (isOurAllianceActive()) return false;
    double s = getSecondsUntilOurNextShift();
    return s > 3.0 && s <= 5.0;
  }

  /** True when 3-0 seconds before our shift. Pre-aim and pre-spool! */
  @Override
  public boolean isGreenLightPreShift() {
    if (isOurAllianceActive()) return false;
    double s = getSecondsUntilOurNextShift();
    return s > 0.0 && s <= 3.0;
  }

  public Alliance getFirstActiveAlliance() {
    return firstActiveAlliance;
  }

  public double getTimeRemainingActive() {
    if (!isOurAllianceActive()) return 0;
    double t = DriverStation.getMatchTime();
    boolean weAreFirst = (alliance == firstActiveAlliance);
    switch (currentPhase) {
      case SHIFT_1:
        return weAreFirst ? Math.max(0, t - 105) : Math.max(0, t - 80);
      case SHIFT_2:
        return weAreFirst ? Math.max(0, t - 80) : Math.max(0, t - 55);
      case SHIFT_3:
        return weAreFirst ? Math.max(0, t - 55) : Math.max(0, t - 30);
      case SHIFT_4:
        return weAreFirst ? Math.max(0, t - 30) : Math.max(0, t);
      case END_GAME:
        return Math.max(0, t);
      default:
        return 0;
    }
  }
}
