package frc.robot.subsystems.swerve;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.config.SwerveModuleConfiguration;

import static org.littletonrobotics.junction.LoggedRobot.defaultPeriodSecs;

/**
 * Represents a simulated SwerveModuleIO class.
 */
public class SwerveModuleIOSim implements SwerveModuleIO {
    /**
     * Simulate a drive motor using the accurate gearing ratios and moment of inertia
     */
    private final DCMotorSim driveSim;
    /**
     * Simulate a steer motor using the accurate gearing ratios and moment of inertia
     */
    private final DCMotorSim steerSim;

    /**
     * PID Controller for drive motor. Converts target drive velocity to voltage.
     */
    private final PIDController driveFeedback = new PIDController(0.1, 0, 0);

    private final SimpleMotorFeedforward driveFeedforward;

    /**
     * PID Controller for steer motor. Converts target angular position to voltage.
     */
    private final PIDController steerFeedback = new PIDController(10, 0, 0);

    private final SwerveModuleConfiguration swerveModuleConfig;
    private double driveAppliedVolts = 0.0;
    private double steerAppliedVolts = 0.0;
    private final Rotation2d steerOffset;

    @SuppressWarnings("UnusedVariable")
    private SwerveModuleState state = new SwerveModuleState();

    public SwerveModuleIOSim(SwerveModuleConfiguration swerveModuleConfig) {
        this.swerveModuleConfig = swerveModuleConfig;
        driveSim = new DCMotorSim(DCMotor.getFalcon500(1), swerveModuleConfig.DriveMotorGearRatio, 0.025);
        steerSim = new DCMotorSim(DCMotor.getFalcon500(1), swerveModuleConfig.SteerMotorGearRatio, 0.004096955);
        driveFeedforward = new SimpleMotorFeedforward(0.0, 0.085);
        steerOffset = Rotation2d.fromDegrees(swerveModuleConfig.CANcoderOffsetRotations);
        steerFeedback.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        // Update every period - use a set amount of seconds.
        driveSim.update(defaultPeriodSecs);
        steerSim.update(defaultPeriodSecs);

        inputs.timestamp = HALUtil.getFPGATime() / 1.0e6;
        inputs.drivePositionRad = driveSim.getAngularPositionRad();
        inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

        inputs.steerAbsolutePosition =
                Rotation2d.fromRadians(steerSim.getAngularPositionRad()).plus(steerOffset);
        inputs.steerPosition = Rotation2d.fromRadians(steerSim.getAngularPositionRad());
        inputs.steerPositionRad = steerSim.getAngularPositionRad();
        inputs.steerVelocityRadPerSec = steerSim.getAngularVelocityRadPerSec();
        inputs.steerPositionDeg = Math.toDegrees(inputs.steerPositionRad);
        inputs.steerAppliedVolts = steerAppliedVolts;
        inputs.steerCurrentDrawAmps = Math.abs(steerSim.getCurrentDrawAmps());

        inputs.targetDriveVelocityMetersPerSec = driveFeedback.getSetpoint();
        inputs.targetSteerPositionRad = steerFeedback.getSetpoint();
    }

    @Override
    public void setModuleState(SwerveModuleState state, boolean steerMotionMagicEnabled) {
        this.state = state;

        // Calculate target data to voltage data
        var velocityRadPerSec = state.speedMetersPerSecond / swerveModuleConfig.WheelRadiusInMeters;

        driveAppliedVolts = driveFeedforward.calculate(velocityRadPerSec)
                + driveFeedback.calculate(driveSim.getAngularVelocityRadPerSec(), velocityRadPerSec);

        driveAppliedVolts = MathUtil.clamp(driveAppliedVolts, -12.0, 12.0);

        steerAppliedVolts = steerFeedback.calculate(getAngle().getRadians(), state.angle.getRadians());
        steerAppliedVolts = MathUtil.clamp(steerAppliedVolts, -12.0, 12.0);

        // Apply calculated voltage data to simulated motors
        driveSim.setInputVoltage(driveAppliedVolts);
        steerSim.setInputVoltage(steerAppliedVolts);
    }

    public Rotation2d getAngle() {
        if (steerOffset == null) {
            return new Rotation2d();
        } else {
            return Rotation2d.fromRadians(steerSim.getAngularPositionRad()).plus(steerOffset);
        }
    }
}
