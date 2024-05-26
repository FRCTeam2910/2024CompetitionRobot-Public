package frc.robot.config;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.MacAddressUtil;

import static frc.robot.util.MacAddressUtil.getMACAddress;

public enum RobotIdentity {
    TYPHOON,
    LOKI,
    SIMULATION;

    public static RobotIdentity getIdentity() {
        if (!Robot.isReal()) {
            return SIMULATION;
        } else {
            String mac = getMACAddress();
            if (!mac.isEmpty()) {
                if (mac.equals(MacAddressUtil.ROBOT_ONE_MAC)) {
                    return LOKI;
                }
            }

            return TYPHOON;
        }
    }

    public static Constants.Mode getMode() {
        switch (getIdentity()) {
            case LOKI:
            case TYPHOON:
                return Robot.isReal() ? Constants.Mode.REAL : Constants.Mode.REPLAY;
            case SIMULATION:
                return Constants.Mode.SIM;
            default:
                return Constants.Mode.REAL;
        }
    }
}
