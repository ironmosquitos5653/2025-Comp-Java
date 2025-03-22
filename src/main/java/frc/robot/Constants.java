// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

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

  public static enum Position {

    // Elevator Positions
    ELV_off(0, 0, 0),
    ELV_1(0, .735, 0),
    ELV_2(5.8, .7, 0),
    ELV_3(15.4, .7, 0),
    ELV_4(30, .71, 0),
    ELV_Intake(0, .82, 0),
    ELV_IntakeTravel(0, .9, 0),
    ELV_4_algae(18, .58, .32),
    ELV_algaeLow(10.4, .58, .32),
    ELV_4_auto(30, .54, 0),

    // Algae Spit
    Algae_Spit(30, 0, .08),

    Algae_tmp(0, 0, 0),

    // Climb Positions,
    Climb_Out(0, 0, 0),
    Climb_In(0, 0, 0);

    public double position;
    public double angle;
    public double algae;

    Position(double p, double a, double algae) {
      position = p;
      angle = a;
      this.algae = algae;
    }

    public double angle() {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'angle'");
    }
  }
}
