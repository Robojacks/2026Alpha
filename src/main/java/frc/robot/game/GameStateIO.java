// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.game;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import frc.robot.game.GameState.GamePhase;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface GameStateIO {
  class GameStateInputs implements LoggableInputs {
    public Alliance alliance = Alliance.Blue;
    public Alliance firstActiveAlliance = null;
    public GamePhase phase = GamePhase.PRE_MATCH;
    public boolean shouldHeadBack = false;
    public boolean shouldStartShooting = false;
    public MatchType matchType = MatchType.None;

    @Override
    public void toLog(LogTable table) {
      table.put("Alliance", toName(alliance));
      table.put("FirstActiveAlliance", toName(firstActiveAlliance));
      table.put("Phase", phase.name());
      table.put("ShouldHeadBack", shouldHeadBack);
      table.put("ShouldStartShooting", shouldStartShooting);
      table.put("MatchType", matchType.name());
    }

    @Override
    public void fromLog(LogTable table) {
      alliance = parseAlliance(table.get("Alliance", toName(Alliance.Blue)), Alliance.Blue);
      firstActiveAlliance = parseAlliance(table.get("FirstActiveAlliance", "None"), null);
      phase = parsePhase(table.get("Phase", GamePhase.PRE_MATCH.name()), GamePhase.PRE_MATCH);
      shouldHeadBack = table.get("ShouldHeadBack", false);
      shouldStartShooting = table.get("ShouldStartShooting", false);
      matchType = parseMatchType(table.get("MatchType", MatchType.None.name()), MatchType.None);
    }

    private static String toName(Alliance alliance) {
      return alliance == null ? "None" : alliance.name();
    }

    private static Alliance parseAlliance(String value, Alliance fallback) {
      if (value == null || value.isBlank() || value.equalsIgnoreCase("None")) {
        return fallback;
      }
      if (value.equalsIgnoreCase(Alliance.Blue.name())) {
        return Alliance.Blue;
      }
      if (value.equalsIgnoreCase(Alliance.Red.name())) {
        return Alliance.Red;
      }
      return fallback;
    }

    private static GamePhase parsePhase(String value, GamePhase fallback) {
      try {
        return GamePhase.valueOf(value);
      } catch (IllegalArgumentException ex) {
        return fallback;
      }
    }

    private static MatchType parseMatchType(String value, MatchType fallback) {
      try {
        return MatchType.valueOf(value);
      } catch (IllegalArgumentException ex) {
        return fallback;
      }
    }
  }

  default void updateInputs(GameStateInputs inputs) {}

  default boolean isHeadBackWarning() {
    return false;
  }

  default boolean isGreenLightPreShift() {
    return false;
  }
}
