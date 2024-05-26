// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final Mode currentMode = Mode.REAL;
    public static final int PRIMARY_XBOX_CONTROLLER_PORT = 0;
    public static final int OPERATOR_XBOX_CONTROLLER_PORT = 1;
    public static final int SIMULATOR_XBOX_CONTROLLER_PORT = 2;

    public static final boolean SILENCE_JOYSTICK_WARNINGS_IN_SIMULATOR = true;
    public static final String OPERATOR_DASHBOARD_NAME = "Dashboard";

    public static final double SMART_AUTO_WAIT_FOR_COLLECTION_DELAY = 0.2;

    public enum Mode {
        /**
         * Running on a real robot.
         */
        REAL,

        /**
         * Running a physics simulator.
         */
        SIM,

        /**
         * Replaying from a log file.
         */
        REPLAY
    }
}
