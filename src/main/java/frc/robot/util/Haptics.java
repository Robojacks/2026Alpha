// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Haptics {
  private final GenericHID[] controllers;

  public Haptics(GenericHID... controllers) {
    this.controllers = controllers;
  }

  public Command headBackWarningCommand() {
    return rumblePulse(0.6, 0.4);
  }

  public Command startShootingCommand() {
    return rumblePulse(1.0, 0.5);
  }

  private Command rumblePulse(double intensity, double seconds) {
    return Commands.startEnd(() -> setRumble(intensity), () -> setRumble(0.0)).withTimeout(seconds);
  }

  private void setRumble(double intensity) {
    for (GenericHID controller : controllers) {
      if (controller != null) {
        controller.setRumble(RumbleType.kBothRumble, intensity);
      }
    }
  }
}
