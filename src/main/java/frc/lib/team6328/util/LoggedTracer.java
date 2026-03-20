// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.team6328.util;

import org.littletonrobotics.junction.Logger;

public final class LoggedTracer {
  private LoggedTracer() {}

  public static void record(String name) {
    Logger.recordOutput("Traces/" + name, System.nanoTime());
  }
}
