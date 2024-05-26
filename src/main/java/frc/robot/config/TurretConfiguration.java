package frc.robot.config;

import frc.robot.util.drivers.CanDeviceId;

@SuppressWarnings("InconsistentCapitalization")
public class TurretConfiguration {
    public ServoMotorConfiguration TurretServoMotorConfiguration = new ServoMotorConfiguration();
    public CanDeviceId CanCoderG1CanDeviceId = new CanDeviceId(-1);
    public CanDeviceId CanCoderG2CanDeviceId = new CanDeviceId(-1);
    public double EncodeOffsetG1Rotations = 0.0;
    public double EncodeOffsetG2Rotations = 0.0;
    public boolean g1Inverted = false;
    public boolean g2Inverted = false;

    public TurretConfiguration withTurretMotor(ServoMotorConfiguration turretServoMotorConfiguration) {
        this.TurretServoMotorConfiguration = turretServoMotorConfiguration;
        return this;
    }

    public TurretConfiguration withCanCoderG1CanDeviceId(CanDeviceId canDeviceId) {
        this.CanCoderG1CanDeviceId = canDeviceId;
        return this;
    }

    public TurretConfiguration withCanCoderG2CanDeviceId(CanDeviceId canDeviceId) {
        this.CanCoderG2CanDeviceId = canDeviceId;
        return this;
    }

    public TurretConfiguration withEncodeOffserG1Rotations(double encodeOffsetRotations) {
        this.EncodeOffsetG1Rotations = encodeOffsetRotations;
        return this;
    }

    public TurretConfiguration withEncodeOffserG2ROtations(double encodeOffsetRotations) {
        this.EncodeOffsetG2Rotations = encodeOffsetRotations;
        return this;
    }

    /** CCW is positive
     * */
    public TurretConfiguration withG1Inverted(boolean inverted) {
        this.g1Inverted = inverted;
        return this;
    }
    /** CCW is positive
     * */
    public TurretConfiguration withG2Inverted(boolean inverted) {
        this.g2Inverted = inverted;
        return this;
    }
}
