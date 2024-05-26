package frc.robot.config;

import frc.robot.util.drivers.CanDeviceId;

public class PortConfiguration {
    public int candleID;
    public String CANBus;
    public CanDeviceId teacupMotorCanDeviceID;
    public CanDeviceId rightFeederMotorID;
    public CanDeviceId leftFeederMotorID;
    public CanDeviceId intakeTopMotorID;
    public CanDeviceId intakeBottomMotorID;
    public CanDeviceId shooterLeftMotorID;
    public CanDeviceId shooterRightMotorID;
    public CanDeviceId climberID;
    public CanDeviceId pivotID;
    public CanDeviceId ampBarID;
    public int beamBreakDIOId;

    public PortConfiguration withCandleID(int candleID) {
        this.candleID = candleID;
        return this;
    }

    public PortConfiguration withCANBus(String CANBus) {
        this.CANBus = CANBus;
        return this;
    }
}
