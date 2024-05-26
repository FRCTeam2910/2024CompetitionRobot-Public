package frc.robot.subsystems.turret;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.config.TurretConfiguration;
import frc.robot.subsystems.servo.ServoIO;
import frc.robot.subsystems.servo.ServoSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;

/**
 * Represents a turret. The turret is used to aim the shooter at a target.
 * It is a servo-mechanism that can rotate between certain angles.
 *
 * <p>The turret a single Kraken motor using two absolute encoders in a differential setup to determine the
 * absolute heading.  This is required as we're planning for the turret to be able to rotate 360 degrees in
 * either direction.
 */
public class TurretSubsystem extends ServoSubsystem {
    private static final double GEAR_0_TOOTH_COUNT = 70.0;
    private static final double GEAR_1_TOOTH_COUNT = 36.0;
    private static final double GEAR_2_TOOTH_COUNT = 34.0;

    private static final double SLOPE = (GEAR_2_TOOTH_COUNT * GEAR_1_TOOTH_COUNT)
            / ((GEAR_1_TOOTH_COUNT - GEAR_2_TOOTH_COUNT) * GEAR_0_TOOTH_COUNT);

    /**
     * This is a ratio comparing turret position error per radian per second.
     */
    private static final double TURRET_POSITION_ERROR_TO_DRIVEBASE_VELOCITY_PROPORTION = 0.213;

    private final CANCoderIOInputsAutoLogged inputsG1 = new CANCoderIOInputsAutoLogged();
    private final CANCoderIOInputsAutoLogged inputsG2 = new CANCoderIOInputsAutoLogged();
    private final CANCoderIO encoderIOG1;
    private final CANCoderIO encoderIOG2;
    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;

    /**
     * Actual desired turret angle.
     */
    private double desiredTurretAngleRadians = Math.PI;
    /**
     * Target turret angle that is offset to compensate for drivetrain theta velocity.
     */
    private double offsetTargetTurretAngleRadians = Math.PI;

    private final BooleanSupplier climberHomedSupplier;
    private final BooleanSupplier climberStowedSupplier;
    private boolean shouldVelocityCompensationBeApplied = true;

    /**
     * Creates a new TurretSubsystem.
     *
     * @param io The TurretIO object that this subsystem will interface with.
     */
    public TurretSubsystem(
            ServoIO io,
            TurretConfiguration config,
            CANCoderIO encoderIOG1,
            CANCoderIO encoderIOG2,
            BooleanSupplier climberHomedSupplier,
            BooleanSupplier climberStowedSupplier) {
        super(io, config.TurretServoMotorConfiguration);
        this.encoderIOG1 = encoderIOG1;
        this.encoderIOG2 = encoderIOG2;
        this.climberHomedSupplier = climberHomedSupplier;
        this.climberStowedSupplier = climberStowedSupplier;
        changeTalonConfig(cfg -> {
            cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            return cfg;
        });

        // Starting positions
        encoderIOG1.updateInputs(inputsG1);
        encoderIOG2.updateInputs(inputsG2);

        Logger.processInputs("CanCoderG1", inputsG1);
        Logger.processInputs("CanCoderG2", inputsG2);

        // Calculate the start angle based on the two encoders
        double startAngle = calculateTurretAngleFromCANCoderDegrees(inputsG1.positionDegrees, inputsG2.positionDegrees);
        Logger.recordOutput("Turret/AlgorithmOutput", startAngle);
        // Set the sensor position to the start angle
        setSensorPosition(unitsToRotations(startAngle));
    }

    /**
     * Sets the robot-relative target angle for the turret.
     * First the closest path from current turret angle to the target angle is calculated.
     * If the path is found to be move outside the bounds, the path will adjust to follow the next closest path.
     *
     * @param targetAngleDegrees Target angle in degrees
     * @param current Current turret angle
     *
     * @return next absolute angle in degrees for the robot to move to
     */
    public static double convertToClosestBoundedTurretAngleDegrees(
            double targetAngleDegrees, Rotation2d current, double forwardLimitDegrees, double reverseLimitDegrees) {
        double currentTotalRadians = (current.getRotations() * 2 * Math.PI);
        double closestOffset = Units.degreesToRadians(targetAngleDegrees) - current.getRadians();
        if (closestOffset > Math.PI) {

            closestOffset -= 2 * Math.PI;

        } else if (closestOffset < -Math.PI) {
            closestOffset += 2 * Math.PI;
        }

        double finalOffset = currentTotalRadians + closestOffset;
        if ((currentTotalRadians + closestOffset) % (2 * Math.PI)
                == (currentTotalRadians - closestOffset)
                        % (2 * Math.PI)) { // If the offset can go either way, go closer to zero
            if (finalOffset > 0) {
                finalOffset = currentTotalRadians - Math.abs(closestOffset);
            } else {
                finalOffset = currentTotalRadians + Math.abs(closestOffset);
            }
        }
        if (finalOffset > Units.degreesToRadians(forwardLimitDegrees)) { // if past upper rotation limit
            finalOffset -= (2 * Math.PI);
        } else if (finalOffset < Units.degreesToRadians(reverseLimitDegrees)) { // if below lower rotation limit
            finalOffset += (2 * Math.PI);
        }

        return Units.radiansToDegrees(finalOffset);
    }

    public static double calculateTurretAngleFromCANCoderDegrees(double e1, double e2) {
        double difference = e2 - e1;
        if (difference > 250) {
            difference -= 360;
        }
        if (difference < -250) {
            difference += 360;
        }
        difference *= SLOPE;

        double e1Rotations = (difference * GEAR_0_TOOTH_COUNT / GEAR_1_TOOTH_COUNT) / 360.0;
        double e1RotationsFloored = Math.floor(e1Rotations);
        double turretAngle = (e1RotationsFloored * 360.0 + e1) * (GEAR_1_TOOTH_COUNT / GEAR_0_TOOTH_COUNT);
        if (turretAngle - difference < -100) {
            turretAngle += GEAR_1_TOOTH_COUNT / GEAR_0_TOOTH_COUNT * 360.0;
        } else if (turretAngle - difference > 100) {
            turretAngle -= GEAR_1_TOOTH_COUNT / GEAR_0_TOOTH_COUNT * 360.0;
        }
        return turretAngle;
    }

    @Override
    public void periodic() {
        super.periodic();
        encoderIOG1.updateInputs(inputsG1);
        encoderIOG2.updateInputs(inputsG2);

        Logger.processInputs("CanCoderG1", inputsG1);
        Logger.processInputs("CanCoderG2", inputsG2);

        double canCoderAngle =
                calculateTurretAngleFromCANCoderDegrees(inputsG1.positionDegrees, inputsG2.positionDegrees);
        Logger.recordOutput("Turret/AlgorithmOutput", canCoderAngle);

        if (DriverStation.isTeleop()) {
            if (!climberHomedSupplier.getAsBoolean() || !climberStowedSupplier.getAsBoolean()) {
                shouldVelocityCompensationBeApplied = false;
                offsetTargetTurretAngleRadians = 0.0;
                wantedState = WantedState.TARGET;
            } else {
                shouldVelocityCompensationBeApplied = true;
            }
        }

        // process inputs
        SystemState newState = handleStateTransition();
        if (newState != systemState) {
            Logger.recordOutput("Turret/SystemState", newState.toString());
            systemState = newState;
        }

        if (systemState == SystemState.RESETTING) {
            handleResetting();
        }

        // Push the current turret angle to the robot state
        RobotState.getInstance()
                .addTurretAngleObservation(
                        new RobotState.TurretAngleObservation(Timer.getFPGATimestamp(), getTurretAngle()));

        // write outputs
        if (systemState != SystemState.IDLING) {
            setSetpointPositionPID(Units.radiansToDegrees(offsetTargetTurretAngleRadians));
        } else {
            this.stop();
        }

        var positionAngleDegrees = rotationsToHomedUnits(inputs.positionRotations);
        var setpointDegrees = Rotation2d.fromRadians(desiredTurretAngleRadians).getDegrees();
        var setpointDegreesWithOffset =
                Rotation2d.fromRadians(offsetTargetTurretAngleRadians).getDegrees();
        var error = positionAngleDegrees - setpointDegrees;
        var turretErrorToGyroAngularVelocityProportion =
                error / Units.radiansToDegrees(RobotState.getInstance().getLastGyroAngularVelocity());
        Logger.recordOutput("Turret/PositionAngleDegrees", positionAngleDegrees);
        Logger.recordOutput("Turret/DesiredTurretAngleDegrees", setpointDegrees);
        Logger.recordOutput("Turret/OffsetTargetTurretAngle", setpointDegreesWithOffset);
        Logger.recordOutput("Turret/AngleErrorDegrees", error);
        Logger.recordOutput("Turret/ErrorPerDriveRotationVelocity", turretErrorToGyroAngularVelocityProportion);
    }

    private void handleResetting() {
        desiredTurretAngleRadians = 0.0;
    }

    public Rotation2d getTurretAngle() {
        return Rotation2d.fromDegrees(rotationsToHomedUnits(inputs.positionRotations));
    }

    private SystemState handleStateTransition() {
        if (wantedState == WantedState.TARGET) {
            if (!atSetpoint()
                    && systemState == SystemState.RESETTING) { // Changed condition from atTrajectory to atSetpoint
                return SystemState.RESETTING;
            } else {
                return SystemState.TARGETING;
            }
        } else if (wantedState == WantedState.RESET) {
            return SystemState.RESETTING;
        }

        return SystemState.IDLING;
    }

    /**
     * Set the setpoint for the turret in radians.
     *
     * @param targetAngleDegrees the target angle in degrees
     */
    public double convertToClosestBoundedTurretAngleRadians(double targetAngleDegrees) {
        return Units.degreesToRadians(convertToClosestBoundedTurretAngleDegrees(
                targetAngleDegrees, getTurretAngle(), forwardMaxUnitsLimit, reverseMinUnitsLimit));
    }

    /**
     * Set the wanted state of the turret.
     */
    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    /**
     * Set the wanted state of the turret with a Rotation2D that will become the target angle of the system.
     */
    public void setWantedState(WantedState wantedState, Rotation2d angle) {
        this.wantedState = wantedState;
        desiredTurretAngleRadians = convertToClosestBoundedTurretAngleRadians(angle.getDegrees());
        if (shouldVelocityCompensationBeApplied) {
            offsetTargetTurretAngleRadians = convertToClosestBoundedTurretAngleRadians(angle.getDegrees()
                    + (TURRET_POSITION_ERROR_TO_DRIVEBASE_VELOCITY_PROPORTION
                            * Units.radiansToDegrees(RobotState.getInstance().getLastGyroAngularVelocity())));
        } else {
            offsetTargetTurretAngleRadians = desiredTurretAngleRadians;
        }
    }

    public void setShouldVelocityCompensationBeApplied(boolean shouldVelocityCompensationBeApplied) {
        this.shouldVelocityCompensationBeApplied = shouldVelocityCompensationBeApplied;
    }

    public void setWantedStateWithAbsoluteAngle(WantedState wantedState, Rotation2d angle) {
        this.wantedState = wantedState;
        desiredTurretAngleRadians = angle.getDegrees();
        offsetTargetTurretAngleRadians = desiredTurretAngleRadians;
    }

    public boolean atSetpoint() {
        return MathUtil.isNear(getSetpoint(), getPosition(), 2.0);
    }

    public enum WantedState {
        IDLE,
        TARGET,
        RESET
    }

    public enum SystemState {
        IDLING,
        TARGETING,
        RESETTING
    }
}
