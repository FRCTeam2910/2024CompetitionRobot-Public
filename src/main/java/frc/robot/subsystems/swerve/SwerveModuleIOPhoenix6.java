package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.SwerveModuleConfiguration;
import frc.robot.util.drivers.CanDeviceId;
import frc.robot.util.drivers.Phoenix6Util;
import frc.robot.util.drivers.TalonFXFactory;

/**
 * Represents a swerve module implemented using CTR Electronics Phoenix V6 API.
 * It uses TalonFX, i.e. Falcon500 or Kraken60, motors and CANCoders for control.
 */
public class SwerveModuleIOPhoenix6 implements SwerveModuleIO {
    // Hardware objects
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder steerEncoder;

    // Status signals used for synchronizing odometry.
    private final StatusSignal<Double> drivePositionStatusSignal;
    private final StatusSignal<Double> driveVelocityStatusSignal;
    private final StatusSignal<Double> driveAppliedVolts;
    private final StatusSignal<Double> driveSupplyCurrent;
    private final StatusSignal<Double> driveTemperature;

    private final StatusSignal<Double> steerPositionStatusSignal;
    private final StatusSignal<Double> steerVelocityStatusSignal;
    private final StatusSignal<Double> steerAppliedVolts;
    private final StatusSignal<Double> steerSupplyCurrent;
    private final StatusSignal<Double> steerTemperature;
    private final StatusSignal<Double> steerEncoderPositionStatusSignal;
    private final StatusSignal<Double> steerEncoderAbsolutePosition;

    // Control mode setters
    private final VoltageOut driveControlSetter;
    private final PositionVoltage steerControlSetter;
    private final MotionMagicVoltage motionMagicSetter;

    // variables
    private final double driveRotationsPerMeter;

    @SuppressWarnings("UnusedVariable")
    private final double couplingRatioDriveRotorToCANcoder;

    private final SwerveModulePosition internalState = new SwerveModulePosition();

    @SuppressWarnings("UnusedVariable")
    private final boolean supportsPro;

    private final double maxVelocityMetersPerSecond;
    private SwerveModuleConfiguration swerveModuleConfig;

    // Target Variables. Used only for data logging
    private double targetVelocityMetersPerSeconds = 0.0;
    private double targetSteerAngleRadians = 0.0;

    private final double VELOCITY_COEFFICIENT = 1.10;

    /**
     * Initializes the motors, encoder, and the settings for each of the devices.
     *
     * @param swerveModuleConfig The configuration for the swerve module.
     * @param supportsPro        Whether the motor controller supports Pro mode.
     */
    public SwerveModuleIOPhoenix6(SwerveModuleConfiguration swerveModuleConfig, boolean supportsPro) {
        this.swerveModuleConfig = swerveModuleConfig;
        this.supportsPro = supportsPro;

        driveMotor = TalonFXFactory.createDefaultTalon(swerveModuleConfig.DriveCanDeviceId);
        steerMotor = TalonFXFactory.createDefaultTalon(swerveModuleConfig.SteerCanDeviceId);
        steerEncoder = new CANcoder(
                swerveModuleConfig.CancoderCanDeviceId.getDeviceNumber(),
                swerveModuleConfig.CancoderCanDeviceId.getBus());

        // Configure Drive motor
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.Slot0 = swerveModuleConfig.DriveMotorGains;

        //        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = swerveModuleConfig.DriveMotorSupplyCurrent;
        //        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -swerveModuleConfig.DriveMotorSupplyCurrent;

        driveConfig.CurrentLimits.SupplyCurrentLimit = swerveModuleConfig.DriveMotorSupplyCurrent;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = swerveModuleConfig.EnableDriveMotorSupplyCurrentLimit;

        driveConfig.CurrentLimits.StatorCurrentLimit = swerveModuleConfig.DriveMotorStatorCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = swerveModuleConfig.EnableDriveMotorStatorCurrentLimit;

        driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;

        driveConfig.MotorOutput.Inverted = swerveModuleConfig.DriveMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //        StatusCode response = driveMotor.getConfigurator().apply(driveConfig);
        //        if (!response.isOK()) {
        //            System.out.println("Talon ID " + swerveModuleConfig.CancoderCanDeviceId.getDeviceNumber()
        //                    + " failed config with error " + response.toString());
        //        }
        Phoenix6Util.applyAndCheckConfiguration(driveMotor, driveConfig);

        // Configure Steer motor
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();

        steerConfig.Slot0 = swerveModuleConfig.SteerMotorGains;

        steerConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / swerveModuleConfig.SteerMotorGearRatio;
        steerConfig.MotionMagic.MotionMagicAcceleration = steerConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
        steerConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * swerveModuleConfig.SteerMotorGearRatio;
        steerConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
        steerConfig.Slot1.kP = 16;
        steerConfig.Slot1.kI = 0;
        steerConfig.Slot1.kD = 0;
        steerConfig.Slot1.kS = 0.8;
        steerConfig.Slot1.kV = 0.1224;

        // CANcoder Configuration, apply offset.
        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.MagnetOffset = swerveModuleConfig.CANcoderOffsetRotations;

        var response = steerEncoder.getConfigurator().apply(cancoderConfigs);

        if (!response.isOK()) {
            System.out.println("CANcoder ID " + swerveModuleConfig.CancoderCanDeviceId.getDeviceNumber()
                    + " failed config with error " + response.toString());
        }

        // Modify configuration to use remote CANcoder fused
        steerConfig.Feedback.FeedbackRemoteSensorID = swerveModuleConfig.CancoderCanDeviceId.getDeviceNumber();
        switch (swerveModuleConfig.FeedbackSource) {
            case RemoteCANcoder:
                steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
                break;
            case FusedCANcoder:
                steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
                break;
            case SyncCANcoder:
                steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
                break;
        }
        steerConfig.Feedback.RotorToSensorRatio = swerveModuleConfig.SteerMotorGearRatio;

        steerConfig.CurrentLimits.SupplyCurrentLimit = swerveModuleConfig.SteerMotorSupplyCurrent;
        steerConfig.CurrentLimits.SupplyCurrentLimitEnable = swerveModuleConfig.EnableSteerMotorSupplyCurrentLimit;
        steerConfig.CurrentLimits.StatorCurrentLimit = swerveModuleConfig.SteerMotorStatorCurrent;
        steerConfig.CurrentLimits.StatorCurrentLimitEnable = swerveModuleConfig.EnableSteerMotorStatorCurrentLimit;

        steerConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;

        steerConfig.ClosedLoopGeneral.ContinuousWrap = true; // Enable continuous wrap for swerve modules

        steerConfig.MotorOutput.Inverted = swerveModuleConfig.SteerMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //        response = steerMotor.getConfigurator().apply(steerConfig);
        //        if (!response.isOK()) {
        //            System.out.println("Talon ID " + swerveModuleConfig.SteerCanDeviceId.getDeviceNumber()
        //                    + " failed config with error " + response.toString());
        //        }
        Phoenix6Util.applyAndCheckConfiguration(steerMotor, steerConfig);

        // Get control modes for drive and steer motors
        driveControlSetter = new VoltageOut(0.0).withUpdateFreqHz(0);
        steerControlSetter = new PositionVoltage(0.0).withSlot(0).withUpdateFreqHz(0);
        motionMagicSetter = new MotionMagicVoltage(.00).withSlot(1).withUpdateFreqHz(0);

        // Get signals and set update rate 100hz signals
        driveAppliedVolts = driveMotor.getMotorVoltage();
        driveSupplyCurrent = driveMotor.getSupplyCurrent();
        driveTemperature = driveMotor.getDeviceTemp();
        steerAppliedVolts = steerMotor.getMotorVoltage();
        steerSupplyCurrent = steerMotor.getSupplyCurrent();
        steerTemperature = steerMotor.getDeviceTemp();
        steerEncoderAbsolutePosition = steerEncoder.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                driveAppliedVolts,
                driveSupplyCurrent,
                steerAppliedVolts,
                steerSupplyCurrent,
                driveTemperature,
                steerTemperature,
                steerEncoderAbsolutePosition);

        // Set the status signals to support synchronized odometry.
        drivePositionStatusSignal = driveMotor.getPosition().clone();
        driveVelocityStatusSignal = driveMotor.getVelocity().clone();
        steerPositionStatusSignal = steerMotor.getPosition().clone();
        steerVelocityStatusSignal = steerMotor.getVelocity().clone();
        steerEncoderPositionStatusSignal = steerEncoder.getPosition().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                drivePositionStatusSignal,
                driveVelocityStatusSignal,
                steerPositionStatusSignal,
                steerVelocityStatusSignal,
                steerEncoderPositionStatusSignal);

        BaseStatusSignal.setUpdateFrequencyForAll(50, driveMotor.getFaultField(), steerMotor.getFaultField());

        Phoenix6Odometry.getInstance().registerSignal(driveMotor, drivePositionStatusSignal);
        Phoenix6Odometry.getInstance().registerSignal(driveMotor, driveVelocityStatusSignal);
        Phoenix6Odometry.getInstance().registerSignal(steerMotor, steerPositionStatusSignal);
        Phoenix6Odometry.getInstance().registerSignal(steerMotor, steerVelocityStatusSignal);

        /* Calculate the ratio of drive motor rotation to meter on ground */
        var rotationsPerWheelRotation = swerveModuleConfig.DriveMotorGearRatio;
        var metersPerWheelRotation = 2 * Math.PI * swerveModuleConfig.WheelRadiusInMeters;
        driveRotationsPerMeter = rotationsPerWheelRotation / metersPerWheelRotation;
        couplingRatioDriveRotorToCANcoder = swerveModuleConfig.CouplingGearRatio;
        maxVelocityMetersPerSecond = swerveModuleConfig.SpeedAt12VoltsMetersPerSecond;

        // When optimizing bus utilization, make sure all Signals in use have their update frequency set.
        driveMotor.optimizeBusUtilization();
        steerMotor.optimizeBusUtilization();
        steerEncoder.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.driveMotorConnected = BaseStatusSignal.refreshAll(drivePositionStatusSignal, driveVelocityStatusSignal)
                .isOK();

        inputs.steerMotorConnected = BaseStatusSignal.refreshAll(
                        steerPositionStatusSignal, steerVelocityStatusSignal, steerEncoderPositionStatusSignal)
                .isOK();

        /* Now read the latency-compensate our signals */
        double driveRotations =
                BaseStatusSignal.getLatencyCompensatedValue(drivePositionStatusSignal, driveVelocityStatusSignal);
        double steerRotations =
                BaseStatusSignal.getLatencyCompensatedValue(steerPositionStatusSignal, steerVelocityStatusSignal);

        // refresh the 100 Hz signals.  The other signals area already refreshed in odometry thread.
        BaseStatusSignal.refreshAll(
                driveAppliedVolts,
                driveSupplyCurrent,
                driveTemperature,
                steerAppliedVolts,
                steerSupplyCurrent,
                steerTemperature,
                steerEncoderAbsolutePosition);

        // refresh the 50 Hz signals.
        BaseStatusSignal.refreshAll(driveMotor.getFaultField(), steerMotor.getFaultField());

        inputs.timestamp = HALUtil.getFPGATime() / 1.0e6;

        // Account for gear ratio as we don't apply this in drive motor configuration.
        inputs.drivePositionRad = Units.rotationsToRadians(driveRotations / swerveModuleConfig.DriveMotorGearRatio);
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(
                driveVelocityStatusSignal.getValueAsDouble() / swerveModuleConfig.DriveMotorGearRatio);

        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps = driveSupplyCurrent.getValueAsDouble();

        // Because gear ratio is applied in the steer motor configuration, not need to adjust.
        inputs.steerAbsolutePosition = Rotation2d.fromRotations(steerEncoderAbsolutePosition.getValueAsDouble());
        inputs.steerAbsolutePositionRadians = inputs.steerAbsolutePosition.getRadians();
        inputs.steerPosition = Rotation2d.fromRotations(steerRotations);
        inputs.steerPositionRad = Units.rotationsToRadians(steerRotations);
        inputs.steerVelocityRadPerSec = Units.rotationsToRadians(steerVelocityStatusSignal.getValueAsDouble());
        inputs.steerPositionDeg = Math.toDegrees(inputs.steerPositionRad);

        inputs.steerAppliedVolts = steerAppliedVolts.getValueAsDouble();
        inputs.steerCurrentDrawAmps = steerSupplyCurrent.getValueAsDouble();

        inputs.targetDriveVelocityMetersPerSec = targetVelocityMetersPerSeconds;
        inputs.targetSteerPositionRad = targetSteerAngleRadians;

        inputs.driveTemperature = driveTemperature.getValueAsDouble();
        inputs.steerTemperature = steerTemperature.getValueAsDouble();

        internalState.distanceMeters = driveVelocityStatusSignal.getValueAsDouble() / driveRotationsPerMeter;
        internalState.angle = Rotation2d.fromRotations(inputs.steerPositionRad);

        if (steerMotor.getFaultField().getValue() != 0) {
            Phoenix6Util.checkFaults("Steer" + swerveModuleConfig.Name, steerMotor);
        }
        if (driveMotor.getFaultField().getValue() != 0) {
            Phoenix6Util.checkFaults("Drive" + swerveModuleConfig.Name, driveMotor);
        }
    }

    @Override
    public void setModuleState(SwerveModuleState state, boolean steerMotionMagicEnabled) {
        double angleToSetRotations = state.angle.getRotations();
        if (steerMotionMagicEnabled) {
            steerMotor.setControl(motionMagicSetter.withPosition(angleToSetRotations));
        } else {
            steerMotor.setControl(steerControlSetter.withPosition(angleToSetRotations));
        }

        double velocityToSet = state.speedMetersPerSecond;

        var voltage = (velocityToSet / maxVelocityMetersPerSecond) * 12.0;

        if (DriverStation.isAutonomous()) {
            voltage *= VELOCITY_COEFFICIENT;
        }

        driveMotor.setControl(driveControlSetter.withOutput(voltage));

        // Make these values available for logging
        targetSteerAngleRadians = Units.rotationsToRadians(angleToSetRotations);
        targetVelocityMetersPerSeconds = velocityToSet;
    }

    @Override
    public CanDeviceId getCanDeviceId() {
        return swerveModuleConfig.SteerCanDeviceId;
    }
}
