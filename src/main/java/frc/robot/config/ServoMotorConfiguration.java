package frc.robot.config;

import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.drivers.CanDeviceId;

/**
 * Configuration for a servo motor.
 */
@SuppressWarnings("InconsistentCapitalization")
public class ServoMotorConfiguration {

    public static class TalonFXConstants {
        public CanDeviceId canDeviceId = new CanDeviceId(-1);
        public boolean counterClockwisePositive = true;
        public boolean invertMotorOutput = false;
        public int encoderPpr = 2048;

        public TalonFXConstants withCanDeviceId(CanDeviceId canDeviceId) {
            this.canDeviceId = canDeviceId;
            return this;
        }

        public TalonFXConstants withCounterClockwisePositive(boolean counterClockwisePositive) {
            this.counterClockwisePositive = counterClockwisePositive;
            return this;
        }

        public TalonFXConstants withInvertMotorOutput(boolean invertMotorOutput) {
            this.invertMotorOutput = invertMotorOutput;
            return this;
        }
    }

    /**
     * The name of the subsystem. This is used for logging and debugging.
     */
    public String Name = "ERROR_ASSIGN_A_NAME";

    public double kLooperDt = 0.01;
    public double kCANTimeout = 0.010; // use for important on the fly updates
    public int kLongCANTimeoutMs = 100; // use for constructors

    public TalonFXConstants MasterConstants = new TalonFXConstants();
    public TalonFXConstants[] FollowerConstants = new TalonFXConstants[0];

    /**
     * The state of the motor controller bridge when output is neutral or disabled.
     */
    public NeutralModeValue NeutralMode = NeutralModeValue.Brake;

    /**
     * The position of the mechanism when the motor is at 0.0. This is used for zeroing the mechanism.
     */
    public double HomePosition = 0.0; // Units

    /**
     * This is the ratio of the motor rotations to the mechanism's rotations.
     * This is equivalent to the mechanism's gear ratio.
     */
    public double RotationsPerUnitDistance = 1.0;

    public double SoftLimitDeadband = 0.0;
    public double slot1MotionMagickP = 0; // Raw output / raw error
    public double slot1MotionMagickI = 0; // Raw output / sum of raw error
    public double slot1MotionMagickD = 0; // Raw output / (err - prevErr)
    public double slot1MotionMagickV = 0; // Raw output / velocity rps
    public double slot1MotionMagickA = 0; // Raw output / accel in rps / s
    public double slot1MotionMagickS = 0;
    public int Deadband = 0; // rotations
    public double slot0PositionPIDkP = 0;
    public double slot0PositionPIDkI = 0;
    public double slot0PositionPIDkD = 0;
    public double slot0PositionPIDkS = 0;
    public double slot0PositionPIDkA = 0;

    public int PositionDeadband = 0; // Ticks

    public double slot0PositionPIDkV = 0;
    public double ArbitraryFeedforward = 0;
    public double CruiseVelocity = 0; // Units/s
    public double Acceleration = 0; // Units/s^2
    public double Jerk = 0; // Units/s^3
    public double RampRate = 0.0; // s
    public double MaxVoltage = 12.0;

    /**
     * The amount of supply current allowed. This is only applicable for non-torque current control modes.
     */
    public double SupplyCurrentLimit = 60.0; // amps

    /**
     * Enable the supply current limit. This is only applicable for non-torque current control modes.
     */
    public boolean EnableSupplyCurrentLimit = false;

    /**
     * The amount of current allowed in the motor (motoring and regen current).  This is only applicable for
     * non-torque current controlc modes.
     */
    public double StatorCurrentLimit = 40.0; // amps

    /**
     * Enable the stator current limit. This is only applicable for non-torque current control modes.
     */
    public boolean EnableStatorCurrentLimit = false;

    public double MaxUnitsLimit = Double.POSITIVE_INFINITY;
    public double MinUnitsLimit = Double.NEGATIVE_INFINITY;

    /**
     * This is the ratio of sensor rotations to the mechanism's output.
     * This is equivalent to the mechanism's gear ratio if the sensor is
     * located on the input of a gearbox (motor shaft).  If sensor is on
     * the output of a gearbox, then this is typically set to 1.
     */
    public double SensorToMechanismRatio = 1.0;

    public ServoMotorConfiguration withName(String name) {
        Name = name;
        return this;
    }

    public ServoMotorConfiguration withMasterConstants(TalonFXConstants masterConstants) {
        MasterConstants = masterConstants;
        return this;
    }

    public ServoMotorConfiguration withSlaveConstants(TalonFXConstants[] slaveConstants) {
        FollowerConstants = slaveConstants;
        return this;
    }

    public ServoMotorConfiguration withNeutralMode(NeutralModeValue neutralMode) {
        NeutralMode = neutralMode;
        return this;
    }

    public ServoMotorConfiguration withHomePosition(double homePosition) {
        HomePosition = homePosition;
        return this;
    }

    /**
     * Sets the ratio of the motor rotations to the mechanism in rotations.
     */
    public ServoMotorConfiguration withRotationsPerUnitDistance(double rotationsPerUnitDistance) {
        RotationsPerUnitDistance = rotationsPerUnitDistance;
        return this;
    }

    /**
     * Sets the proportional gain for the motor controller.
     * <p>
     * The units for this gain is dependent on the control mode. Since
     * this gain is multiplied by error in the input, the units should be
     * defined as units of output per unit of input error.
     * @param kP the proportional gain
     * @return this
     */
    public ServoMotorConfiguration withKp(double kP) {
        this.slot1MotionMagickP = kP;
        return this;
    }

    /**
     * Sets the integral gain for the motor controller.
     * <p>
     * The units for this gain is dependent on the control mode. Since
     * this gain is multiplied by error in the input integrated over time
     * (in units of seconds), the units should be defined as units of
     * output per unit of integrated input error. For example, when
     * controlling velocity using a duty cycle closed loop, integrating
     * velocity over time results in rps * s = rotations. Therefore, the
     * units for the integral gain will be duty cycle per rotation of
     * accumulated error, or 1/rot.
     *
     * @param kI the integral gain
     * @return this
     */
    public ServoMotorConfiguration withKi(double kI) {
        this.slot1MotionMagickI = kI;
        return this;
    }

    /**
     * Sets the derivative gain for the motor controller.
     * <p>
     * The units for this gain is dependent on the control mode. Since
     * this gain is multiplied by the derivative of error in the input
     * with respect to time (in units of seconds), the units should be
     * defined as units of output per unit of the differentiated input
     * error. For example, when controlling velocity using a duty cycle
     * closed loop, the derivative of velocity with respect to time is
     * rps/s, which is acceleration. Therefore, the units for the
     * derivative gain will be duty cycle per unit of acceleration error,
     * or 1/(rps/s).
     *
     * @param kD the derivative gain
     * @return this
     */
    public ServoMotorConfiguration withKd(double kD) {
        this.slot1MotionMagickD = kD;
        return this;
    }

    /**
     * Sets the feedforward gain for the motor controller.
     * <p>
     * The units for this gain is dependent on the control mode. Since
     * this gain is multiplied by the requested velocity, the units should
     * be defined as units of output per unit of requested input velocity.
     * For example, when controlling velocity using a duty cycle closed
     * loop, the units for the velocity feed foward gain will be duty
     * cycle per requested rps, or 1/rps.
     *
     * @param kF the feedforward gain
     * @return this
     */
    public ServoMotorConfiguration withKf(double kF) {
        this.slot1MotionMagickV = kF;
        return this;
    }

    /**
     * Sets the acceleration gain for the motor controller.
     * @param kA  the acceleration gain
     * @return  this
     */
    public ServoMotorConfiguration withKa(double kA) {
        this.slot1MotionMagickA = kA;
        return this;
    }

    /**
     * Sets the static gain for the motor controller.
     * @param kS the static gain
     * @return this
     */
    public ServoMotorConfiguration withKs(double kS) {
        this.slot1MotionMagickS = kS;
        return this;
    }

    /**
     * Sets the deadband for the motor controller.
     * @param deadband the deadband
     * @return this
     */
    public ServoMotorConfiguration withDeadband(int deadband) {
        Deadband = deadband;
        return this;
    }

    /**
     * Sets the proportional gain for the position PID controller.
     * @param positionKp the proportional gain for the position PID controller
     * @return this
     */
    public ServoMotorConfiguration withPositionKp(double positionKp) {
        slot0PositionPIDkP = positionKp;
        return this;
    }

    /**
     * Sets the integral gain for the position PID controller.
     * @param positionKi the integral gain for the position PID controller
     * @return this
     */
    public ServoMotorConfiguration withPositionKi(double positionKi) {
        slot0PositionPIDkI = positionKi;
        return this;
    }

    /**
     * Sets the derivative gain for the position PID controller.
     * @param positionKd the derivative gain for the position PID controller
     * @return this
     */
    public ServoMotorConfiguration withPositionKd(double positionKd) {
        slot0PositionPIDkD = positionKd;
        return this;
    }

    /**
     * Sets the feedforward gain for the position PID controller.
     * @param positionKf the feedforward gain for the position PID controller
     * @return this
     */
    public ServoMotorConfiguration withPositionKf(double positionKf) {
        slot0PositionPIDkS = positionKf;
        return this;
    }

    public ServoMotorConfiguration withPositionKa(double positionKa) {
        slot0PositionPIDkA = positionKa;
        return this;
    }

    /**
     * Sets the deadband for the position PID controller.
     * @param positionDeadband the deadband for the position PID controller
     * @return this
     */
    public ServoMotorConfiguration withPositionDeadband(int positionDeadband) {
        PositionDeadband = positionDeadband;
        return this;
    }

    /**
     * Sets the velocity feedforward for the motor controller.
     * @param velocityFeedforward the velocity feedforward
     * @return this
     */
    public ServoMotorConfiguration withVelocityFeedforward(double velocityFeedforward) {
        slot0PositionPIDkV = velocityFeedforward;
        return this;
    }

    /**
     * Sets the arbitrary feedforward for the motor controller.
     * @param arbitraryFeedforward the arbitrary feedforward
     * @return this
     */
    public ServoMotorConfiguration withArbitraryFeedforward(double arbitraryFeedforward) {
        ArbitraryFeedforward = arbitraryFeedforward;
        return this;
    }

    /**
     * Sets the cruise velocity for the motor controller.
     * @param cruiseVelocity the cruise velocity
     * @return this
     */
    public ServoMotorConfiguration withCruiseVelocity(double cruiseVelocity) {
        CruiseVelocity = cruiseVelocity;
        return this;
    }

    /**
     * Sets the acceleration for the motor controller.
     * @param acceleration the acceleration
     * @return this
     */
    public ServoMotorConfiguration withAcceleration(double acceleration) {
        Acceleration = acceleration;
        return this;
    }

    /**
     * Sets the jerk for the motor controller.
     * @param jerk the jerk
     * @return this
     */
    public ServoMotorConfiguration withJerk(double jerk) {
        Jerk = jerk;
        return this;
    }

    /**
     *  Sets the ramp rate for the motor controller.
     * @param rampRate the ramp rate
     * @return this
     */
    public ServoMotorConfiguration withRampRate(double rampRate) {
        RampRate = rampRate;
        return this;
    }

    /**
     * Sets the maximum voltage for the motor controller.
     * @param maxVoltage the maximum voltage
     * @return this
     */
    public ServoMotorConfiguration withMaxVoltage(double maxVoltage) {
        MaxVoltage = maxVoltage;
        return this;
    }

    /**
     * The amount of supply current allowed. This is only applicable for non-torque current control modes.
     * @param supplyCurrentLimit the supply current limit
     * @return this
     */
    public ServoMotorConfiguration withSupplyCurrentLimit(int supplyCurrentLimit) {
        SupplyCurrentLimit = supplyCurrentLimit;
        return this;
    }

    /**
     * Enable the supply current limit. This is only applicable for non-torque current control modes.
     * @param enableSupplyCurrentLimit enable the supply current limit
     * @return this
     */
    public ServoMotorConfiguration withEnableSupplyCurrentLimit(boolean enableSupplyCurrentLimit) {
        EnableSupplyCurrentLimit = enableSupplyCurrentLimit;
        return this;
    }

    /**
     * The amount of current allowed in the motor (motoring and regen current).  This is only applicable for
     * @param statorCurrentLimit the stator current limit
     * @return this
     */
    public ServoMotorConfiguration withStatorCurrentLimit(int statorCurrentLimit) {
        StatorCurrentLimit = statorCurrentLimit;
        return this;
    }

    /**
     * Enable the stator current limit. This is only applicable for non-torque current control modes.
     * @param enableStatorCurrentLimit enable the stator current limit
     * @return this
     */
    public ServoMotorConfiguration withEnableStatorCurrentLimit(boolean enableStatorCurrentLimit) {
        EnableStatorCurrentLimit = enableStatorCurrentLimit;
        return this;
    }

    /**
     * Sets the maximum units limit for the motor controller.
     * @param maxUnitsLimit the maximum units limit
     * @return this
     */
    public ServoMotorConfiguration withMaxUnitsLimit(double maxUnitsLimit) {
        MaxUnitsLimit = maxUnitsLimit;
        return this;
    }

    /**
     * Sets the minimum units limit for the motor controller.
     * @param minUnitsLimit the minimum units limit
     * @return this
     */
    public ServoMotorConfiguration withMinUnitsLimit(double minUnitsLimit) {
        MinUnitsLimit = minUnitsLimit;
        return this;
    }

    /**
     * Sets the soft limit deadband for the motor controller.
     * @param softLimitDeadband the soft limit deadband
     * @return this
     */
    public ServoMotorConfiguration withSoftLimitDeadband(double softLimitDeadband) {
        SoftLimitDeadband = softLimitDeadband;
        return this;
    }

    /**
     * Sets the ratio of sensor rotations to the mechanism's output.
     * @param sensorToMechanismRatio the ratio of sensor rotations to the mechanism's output
     * @return this
     */
    public ServoMotorConfiguration withSensorToMechanismRatio(double sensorToMechanismRatio) {
        SensorToMechanismRatio = sensorToMechanismRatio;
        return this;
    }
}
